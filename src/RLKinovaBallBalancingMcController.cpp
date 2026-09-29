#include "RLKinovaBallBalancingMcController.h"

#include <RBDyn/MultiBodyConfig.h>
#include <Eigen/src/Core/Matrix.h>

#include <mc_rtc/gui/Transform.h>

RLKinovaBallBalancingMcController::RLKinovaBallBalancingMcController(mc_rbdyn::RobotModulePtr rm,
                                                                     double dt,
                                                                     const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{
  config_ = config;
  currentPolicyIndex = size_t(config_("default_policy_index", 0));
  // In-place overload: leaves the member at its default when the key is absent.
  config_("control_frame", controlFrameName);
  // Initialize Constraints
  selfCollisionConstraint->setCollisionsDampers(solver(), {1.2, 70.0});
  solver().removeConstraintSet(dynamicsConstraint);
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
      new mc_solver::DynamicsConstraint(robots(), 0, {diPercent_, dsPercent_, 0.0, 1.2, 100.0}, velPercent_, true));
  solver().addConstraintSet(dynamicsConstraint);
  // Remove the default posture task created by the FSM
  solver().removeTask(getPostureTask(robot().name()));

  initializeRobot();
  initializeRLPolicy();
  configureBallSpawn();
  if(config_.has("sim_ft_sensor"))
  {
    auto c = config_("sim_ft_sensor");
    c("enabled", simFtCompensationEnabled_);
    c("force_gain", simFtForceGain_);
    c("remove_calibration_offset", simFtRemoveCalibOffset_);
  }

  addGui();
  addLog();

  // Kinova Dependant -----
  datastore().make<std::string>("TorqueMode", "Custom");
  datastore().make_call("getPostureTask", [this]() -> mc_tasks::PostureTaskPtr { return postureTask; });
  mc_rtc::log::success("RLKinovaBallBalancingMcController init done");
}

bool RLKinovaBallBalancingMcController::run()
{
  bool run = manageModeSwitching();
  if(byPassQPControl()) // Run RL without taking the QP into account
  {
    return true;
  }
  return run; // Return false if QP fails
}

void RLKinovaBallBalancingMcController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);

  // By now the interface has registered its datastore calls, so a still-negative answer is the real
  // one and worth stating: the compensation silently not firing is indistinguishable from it not
  // being needed, and on hardware it must not fire.
  if(!simFtCompensationLogged_)
  {
    simFtCompensationLogged_ = true;
    if(simFtCompensationEnabled_ && !simulationFtCompensationActive())
    {
      mc_rtc::log::info("[RLKinovaBallBalancingMcController] Real F/T sensor: using its reading as measured");
    }
  }
}

void RLKinovaBallBalancingMcController::initializeRobot()
{
  useQP = config_("policies")[currentPolicyIndex]("use_QP", true);
  isTorqueControl_ = config_("policies")[currentPolicyIndex]("is_torque_control", false);
  if(isTorqueControl_)
  {
    mc_rtc::log::info("[RLKinovaBallBalancingMcController] Using Torque Control mode");
    datastore().make<std::string>("ControlMode", "Torque");
  }
  else
  {
    mc_rtc::log::info("[RLKinovaBallBalancingMcController] Using Position Control mode");
    datastore().make<std::string>("ControlMode", "Position");
  }

  // get the joints order (urdf) depending on the robot used
  robotName_ = robot().name();
  // Remove the floating base joints if they exist
  dofNumber = robot().mb().nrDof();

  q_rl = Eigen::VectorXd::Zero(dofNumber);
  q_zero = Eigen::VectorXd::Zero(dofNumber);
  kp_ = Eigen::VectorXd::Zero(dofNumber);
  kd_ = Eigen::VectorXd::Zero(dofNumber);
  kpBase_ = Eigen::VectorXd::Zero(dofNumber);
  kdBase_ = Eigen::VectorXd::Zero(dofNumber);

  tau_rl_ = Eigen::VectorXd::Zero(dofNumber);

  actionScale = Eigen::VectorXd::Zero(dofNumber);
  currentActionScaled = Eigen::VectorXd::Zero(dofNumber);

  jointNames_ = robot().refJointOrder();

  // Get the gains from the configuration or set default values
  pdGainsRatio_ = config_("policies")[currentPolicyIndex]("pd_gains_ratio", 1.0);
  std::map<std::string, double> actionScale_map = config_("policies")[currentPolicyIndex]("action_scale");
  std::map<std::string, double> kp_map = config_("policies")[currentPolicyIndex]("kp");
  std::map<std::string, double> kd_map = config_("policies")[currentPolicyIndex]("kd");
  std::map<std::string, double> q0_map = config_("policies")[currentPolicyIndex]("q0");

  // Get the default posture target from the robot's posture task
  std::shared_ptr<mc_tasks::PostureTask> FSMPostureTask = getPostureTask(robot().name());
  auto posture = FSMPostureTask->posture();
  int i = 0;
  for(const auto & joint_name : jointNames_)
  {
    mc_rtc::log::info("[RLKinovaBallBalancingMcController] Found joint: {}", joint_name);
    if(const auto & t = posture[robot().jointIndexByName(joint_name)]; !t.empty())
    {
      kpBase_[i] = kp_map.at(joint_name);
      kdBase_[i] = kd_map.at(joint_name);
      q_zero[i] = q0_map.at(joint_name);
      q_rl[i] = t[0];
      actionScale[i] = actionScale_map.at(joint_name);
      i++;
    }
  }

  kp_ = pdGainsRatio_ * kpBase_;
  kd_ = sqrt(pdGainsRatio_) * kdBase_;
  // Initialize Task
  torqueJointTask = std::make_shared<mc_tasks::TorqueJointTask>(solver(), robot().robotIndex(), 100.0, 1);
  torqueJointTask->setStiffness(kp_);
  torqueJointTask->setDamping(kd_);

  wrenchTask = std::make_shared<mc_tasks::WrenchTask>(controlFrameName, robots(), robot().robotIndex(), 500.0);
  wrenchTask_target = sva::ForceVecd::Zero();

  compliantPostureTask = std::make_shared<mc_tasks::CompliantPostureTask>(solver(), robot().robotIndex(), 1, 1);
}

void RLKinovaBallBalancingMcController::initializeRLPolicy()
{
  // load policy specific configuration
  policyPaths_ = config_("policy_path", std::vector<std::string>{"walking_better_h1.onnx"});
  configRL();
  currentObservation = Eigen::VectorXd::Zero(rlPolicy->getObservationSize());
  currentAction = Eigen::VectorXd::Zero(rlPolicy->getActionSize());

  initializeRLObservation();

  for(int i = 0; i < HISTORY_SIZE; ++i)
  {
    jointPos[i] = jointPos[0];
    jointVel[i] = jointVel[0];
    jointAction[i] = jointAction[0];
    eePos[i] = eePos[0];
    eeQuat[i] = eeQuat[0];
    eeLinVel[i] = eeLinVel[0];
    eeAngVel[i] = eeAngVel[0];
    eeWrench[i] = eeWrench[0];
  }
}

void RLKinovaBallBalancingMcController::initializeRLObservation()
{
  auto & real_robot = realRobot(robots()[0].name());
  Eigen::VectorXd q = Eigen::VectorXd::Zero(dofNumber);
  Eigen::VectorXd q_dot = Eigen::VectorXd::Zero(dofNumber);
  const auto & q_mbc = real_robot.mbc().q;
  const auto & q_dot_mbc = real_robot.mbc().alpha;
  size_t i = 0;
  for(const auto & joint_name : jointNames_)
  {
    size_t mbcIndex = real_robot.mb().jointIndexByName(joint_name);
    if(mbcIndex >= q_mbc.size() || mbcIndex >= q_dot_mbc.size())
    {
      mc_rtc::log::error("Joint '{}' not found in the robot's MBC. Skipping.", joint_name);
      continue;
    }
    if(q_mbc[mbcIndex].empty())
    {
      continue;
    }

    q[i] = q_mbc[mbcIndex][0];
    q_dot[i] = q_dot_mbc[mbcIndex][0];
    i++;
  }

  sva::PTransformd eePosW = real_robot.bodyPosW(controlFrameName);
  Eigen::Vector3d eeCurrentPosition = eePosW.translation();

  Eigen::Vector3d eeCurrentLinearVel = real_robot.bodyVelW(controlFrameName).linear();
  Eigen::Vector3d eeCurrentAngularVel = real_robot.bodyVelW(controlFrameName).angular();

  Eigen::Vector4d eeCurrentOrientation = Eigen::Vector4d::Zero();
  // Eigen::Quaterniond quat = Eigen::Quaterniond(eePosW.rotation()).normalized();
  Eigen::Quaterniond quat = Eigen::Quaterniond(eePosW.rotation().transpose()).normalized();
  eeCurrentOrientation << quat.w(), quat.x(), quat.y(), quat.z();

  // Gravity-compensated, in the sensor frame, force first: exactly what the policy's ee_ft_wrench
  // term reproduces in simulation (mode "mc_rtc_calibrated" -- see ball_balancing_mdp.py's
  // ee_ft_mc_rtc_calibrated_wrench, whose docstring names this call). It returns that quantity
  // un-negated, so nothing is flipped here; the negation this used to carry belonged to the raw
  // wrench() reading it was written against.
  const auto & eeForceSensor = real_robot.forceSensor("EEForceSensor");
  sva::ForceVecd wrench = eeForceSensor.wrenchWithoutGravity(real_robot);
  wrench = applySimulationFtResponse(eeForceSensor, wrench);

  // Negated, because mc_rtc and the training pipeline use opposite sign conventions for this
  // sensor. Measured at a static pose with an empty plate (2026-08-18 log): MuJoCo's own force
  // sensor -- the quantity ball_balancing_mdp.py's ee_ft_wrench term consumes -- reads
  // [ 0.1212  2.9803 -0.4245] N there, while mc_rtc reports [-0.1054 -2.9816  0.4170] N. Same
  // magnitudes, every component flipped.
  //
  // The racquet's own weight cancels under either convention, since the calibration was measured
  // in the same one as the sensor it came from, so an empty plate reads ~0 both ways and no static
  // check catches this. What does not cancel is the external load: writing R for MuJoCo's reading
  // and G for the gravity term, training's observation is gain*R - G ~ +R_ball while mc_rtc's
  // wrenchWithoutGravity is -R + G ~ -R_ball. The ball pushes the observation the wrong way.
  //
  // The simulation response above must stay on the mc_rtc-convention quantity -- it reconstructs
  // the raw reading via wrench() and the calibration offset, both of which are in that convention
  // -- so the flip belongs here, after it, and applies on hardware too: the convention being
  // matched is the policy's, not the robot's.
  wrench = -wrench;

  Eigen::Vector6d eeCurrentWrench = Eigen::Vector6d::Zero();
  eeCurrentWrench.head<3>() = wrench.force();
  eeCurrentWrench.tail<3>() = wrench.couple();

  jointPos[0] = q - q_zero;
  jointVel[0] = q_dot;
  jointAction[0] = currentAction;
  eePos[0] = eeCurrentPosition;
  eeQuat[0] = eeCurrentOrientation;
  eeLinVel[0] = eeCurrentLinearVel;
  eeAngVel[0] = eeCurrentAngularVel;
  eeWrench[0] = eeCurrentWrench;

  mc_rtc::log::info("jointPos[0]: {}", jointPos[0].transpose());
  mc_rtc::log::info("jointVel[0]: {}", jointVel[0].transpose());
  mc_rtc::log::info("jointAction[0]: {}", jointAction[0].transpose());
  mc_rtc::log::info("eePos[0]: {}", eePos[0].transpose());
  mc_rtc::log::info("eeQuat[0]: {}", eeQuat[0].transpose());
  mc_rtc::log::info("eeLinVel[0]: {}", eeLinVel[0].transpose());
  mc_rtc::log::info("eeAngVel[0]: {}", eeAngVel[0].transpose());
  mc_rtc::log::info("eeWrench[0]: {}", eeWrench[0].transpose());
}

void RLKinovaBallBalancingMcController::switchPolicy(int policyIndex)
{
  if(policyIndex < 0 || policyIndex >= static_cast<int>(policyPaths_.size()))
  {
    mc_rtc::log::error("Invalid policy index: {}", policyIndex);
    return;
  }

  mc_rtc::log::info("Switching from policy [{}] to policy [{}]", currentPolicyIndex, policyIndex);
  currentPolicyIndex = size_t(policyIndex);

  // Update policy-specific boolean flags
  useQP = config_("policies")[currentPolicyIndex]("use_QP", true);
  isTorqueControl_ = config_("policies")[currentPolicyIndex]("is_torque_control", false);
  if(isTorqueControl_)
    datastore().get<std::string>("ControlMode") = "Torque";
  else
    datastore().get<std::string>("ControlMode") = "Position";

  configRL();

  // Update PD gains
  pdGainsRatio_ = config_("policies")[currentPolicyIndex]("pd_gains_ratio", 1.0);
  std::map<std::string, double> kp_map = config_("policies")[currentPolicyIndex]("kp");
  std::map<std::string, double> kd_map = config_("policies")[currentPolicyIndex]("kd");
  std::map<std::string, double> q0_map = config_("policies")[currentPolicyIndex]("q0");
  std::map<std::string, double> actionScale_map = config_("policies")[currentPolicyIndex]("action_scale");

  for(int i = 0; i < dofNumber; ++i)
  {
    const auto & jName = robot().mb().joint(static_cast<int>(i + 1)).name(); // +1 to skip Root
    if(kp_map.count(jName))
    {
      kpBase_(i) = kp_map[jName];
    }
    if(kd_map.count(jName))
    {
      kdBase_(i) = kd_map[jName];
    }
    if(q0_map.count(jName))
    {
      q_zero[i] = q0_map[jName];
    }
    if(actionScale_map.count(jName))
    {
      actionScale[i] = actionScale_map[jName];
    }
  }
  // Update PD gains
  kp_ = pdGainsRatio_ * kpBase_;
  kd_ = sqrt(pdGainsRatio_) * kdBase_;
  torqueJointTask->setStiffness(kp_);
  torqueJointTask->setDamping(kd_);
}

bool RLKinovaBallBalancingMcController::byPassQPControl()
{
  if(useQP) return false; // QP is not bypassed, do nothing
  if(!isTorqueControl_)
  {
    mc_rtc::log::warning("[RLKinovaBallBalancingMcController] QP can't be bypassed in position control mode.");
    mc_rtc::log::warning("[RLKinovaBallBalancingMcController] QP is enforced.");
    return false;
  }

  robot().forwardKinematics();
  robot().forwardVelocity();
  robot().forwardAcceleration();

  auto & real_robot = realRobot(robots()[0].name());

  int i = 0;
  for(const auto & joint_name : jointNames_)
  {
    double q = real_robot.mbc().q[robot().jointIndexByName(joint_name)][0];
    double q_dot = real_robot.mbc().alpha[robot().jointIndexByName(joint_name)][0];
    tau_rl_(i) = kp_(i) * (q_rl(i) - q) - kd_(i) * q_dot;
    robot().mbc().jointTorque[robot().jointIndexByName(joint_name)][0] = tau_rl_(i);
    i++;
  }
  return true;
}

void RLKinovaBallBalancingMcController::toggleQPControl()
{
  // Which path the policy takes is a property of the state, not a flag that can be flipped under
  // it: the policy only reaches the QP through torqueJointTask, and only the RL state adds that
  // task and feeds it q_rl. Flipping useQP inside RLWithoutQP therefore handed the arm to a QP
  // with nothing tracking the policy -- the actions kept being computed and went nowhere, and the
  // arm drifted away for as long as it was left alone (the racquet passed the vertical, pointing
  // at the ground, ~30 s after the toggle). So switch states instead; the two RL states differ in
  // exactly this, and each sets useQP itself on entry.
  //
  // Entering an RL state respawns the ball unless that is unchecked in the GUI, so a switch
  // restarts the balancing attempt rather than continuing the current one.
  const std::string & current = executor_.state();
  if(current == "RLKinovaBallBalancingMcController_RLWithoutQP")
  {
    resume("RLKinovaBallBalancingMcController_RL");
    return;
  }
  if(current == "RLKinovaBallBalancingMcController_RL")
  {
    resume("RLKinovaBallBalancingMcController_RLWithoutQP");
    return;
  }
  // Everywhere else the flag is still meaningful: Initial and the WrenchTask states drive the QP
  // through tasks of their own, so they keep working whichever way it is set.
  useQP = !useQP;
}

void RLKinovaBallBalancingMcController::addLog()
{
  // Robot State variables
  logger().addLogEntry("RLKinovaBallBalancingMcController_kp_base", [this]() { return kpBase_; });
  logger().addLogEntry("RLKinovaBallBalancingMcController_kd_base", [this]() { return kdBase_; });
  logger().addLogEntry("RLKinovaBallBalancingMcController_kp_current", [this]() { return kp_; });
  logger().addLogEntry("RLKinovaBallBalancingMcController_kd_current", [this]() { return kd_; });
  logger().addLogEntry("RLKinovaBallBalancingMcController_pd_gains_ratio", [this]() { return pdGainsRatio_; });

  // RL variables
  logger().addLogEntry("RLKinovaBallBalancingMcController_RL_q", [this]() { return q_rl; });
  logger().addLogEntry("RLKinovaBallBalancingMcController_RL_qZero", [this]() { return q_zero; });
  logger().addLogEntry("RLKinovaBallBalancingMcController_RL_currentObservation",
                       [this]() { return currentObservation; });
  logger().addLogEntry("RLKinovaBallBalancingMcController_RL_currentAction", [this]() { return currentAction; });
  logger().addLogEntry("RLKinovaBallBalancingMcController_RL_tau", [this]() { return tau_rl_; });

  // Controller state variables
  logger().addLogEntry("RLKinovaBallBalancingMcController_useQP", [this]() { return useQP; });
  logger().addLogEntry("RLKinovaBallBalancingMcController_isTorqueControl", [this]() { return isTorqueControl_; });

  // Raw sensor reading alongside the compensated one in eeWrench_0: without it there is no way to
  // tell from a log whether the simulation compensation fired, or what it did.
  logger().addLogEntry("RLKinovaBallBalancingMcController_eeWrench_raw",
                       [this]() -> Eigen::Vector6d
                       {
                         const auto & fs = realRobot(robots()[0].name()).forceSensor("EEForceSensor");
                         Eigen::Vector6d w = Eigen::Vector6d::Zero();
                         w.head<3>() = fs.wrench().force();
                         w.tail<3>() = fs.wrench().couple();
                         return w;
                       });
  logger().addLogEntry("RLKinovaBallBalancingMcController_simFtCompensation",
                       [this]() { return simDetected_ == 1; });

  // Log observation
  for(int i = 0; i < HISTORY_SIZE; ++i)
  {
    logger().addLogEntry("RLKinovaBallBalancingMcController_eePos_" + std::to_string(i),
                         [this, i]() { return eePos[i]; });
    logger().addLogEntry("RLKinovaBallBalancingMcController_eeLinVel_" + std::to_string(i),
                         [this, i]() { return eeLinVel[i]; });
    logger().addLogEntry("RLKinovaBallBalancingMcController_eeAngVel_" + std::to_string(i),
                         [this, i]() { return eeAngVel[i]; });
    logger().addLogEntry("RLKinovaBallBalancingMcController_eeQuat_" + std::to_string(i),
                         [this, i]() { return eeQuat[i]; });
    logger().addLogEntry("RLKinovaBallBalancingMcController_eeWrench_" + std::to_string(i),
                         [this, i]() { return eeWrench[i]; });
  }
}

void RLKinovaBallBalancingMcController::addGui()
{
  gui()->addElement(
      {"RLKinovaBallBalancingMcController", "Policy"},
      mc_rtc::gui::Label("Current policy",
                         [this]() -> const std::string & { return policyPaths_[currentPolicyIndex]; }),
      mc_rtc::gui::ComboInput(
          "Select policy", policyPaths_, [this]() -> const std::string & { return policyPaths_[currentPolicyIndex]; },
          [this](const std::string & selected) { // Capture config by VALUE (makes a safe copy)
            // Find the index of the selected policy
            auto it = std::find(policyPaths_.begin(), policyPaths_.end(), selected);
            if(it != policyPaths_.end())
            {
              int newIndex = static_cast<int>(std::distance(policyPaths_.begin(), it));
              mc_rtc::log::info("User requested policy switch to [{}]: {}", newIndex, selected);
              // Switch to new policy without reinitializing robot
              switchPolicy(newIndex);
            }
          }),
      mc_rtc::gui::Button("Reload current policy",
                          [this]()
                          {
                            mc_rtc::log::info("User requested to reload current policy [{}]", currentPolicyIndex);
                            switchPolicy(int(currentPolicyIndex));
                          }));

  // Add PD gains ratio slider
  gui()->addElement({"RLKinovaBallBalancingMcController", "PD Gains"},
                    mc_rtc::gui::NumberSlider(
                        "PD Gains Ratio", [this]() { return pdGainsRatio_; },
                        [this](double v)
                        {
                          pdGainsRatio_ = v;
                          kp_ = pdGainsRatio_ * kpBase_;
                          kd_ = sqrt(pdGainsRatio_) * kdBase_;
                          torqueJointTask->setStiffness(kp_);
                          torqueJointTask->setDamping(kd_);
                        },
                        0.0, 2.0),
                    mc_rtc::gui::Label("Current kp", kp_), mc_rtc::gui::Label("Current kd", kd_));

  gui()->addElement({"RLKinovaBallBalancingMcController", "Ball"},
                    mc_rtc::gui::Button("Respawn ball on plate", [this]() { spawnBall(); }),
                    mc_rtc::gui::Checkbox(
                        "Respawn on RL state entry", [this]() { return spawnBallOnRLState; },
                        [this]() { spawnBallOnRLState = !spawnBallOnRLState; }));

  gui()->addElement({"ControlMode"},
                    mc_rtc::gui::Button("Switch Control Mode",
                                        [this]()
                                        {
                                          controlModeChanged_ = true;
                                          isTorqueControl_ = !isTorqueControl_;
                                        }),
                    mc_rtc::gui::Label("Current Control Mode",
                                       [this]() { return isTorqueControl_ ? "Torque Control" : "Position Control"; }),
                    mc_rtc::gui::Button("Toggle QP Control", [this]() { toggleQPControl(); }),
                    mc_rtc::gui::Label("QP Control", [this]() { return useQP ? "Enforced" : "Bypassed"; }));
}

void RLKinovaBallBalancingMcController::configRL()
{
  mc_rtc::log::info("Loading RL policy [{}]: {}", currentPolicyIndex, policyPaths_[currentPolicyIndex]);
  try
  {
    rlPolicy = std::make_unique<RLPolicyInterface>(policyPaths_[currentPolicyIndex]);
    if(rlPolicy)
    {
      mc_rtc::log::success("RL policy loaded successfully");
      // Initialize observation vector with the correct size from the loaded policy
      currentObservation = Eigen::VectorXd::Zero(rlPolicy->getObservationSize());
      mc_rtc::log::info("Initialized observation vector with size: {}", rlPolicy->getObservationSize());
      currentAction = Eigen::VectorXd::Zero(rlPolicy->getActionSize());
      mc_rtc::log::info("Initialized action vector with size: {}", rlPolicy->getActionSize());
    }
    else
    {
      mc_rtc::log::error_and_throw("RL policy creation failed - policy is null");
    }
  }
  catch(const std::exception & e)
  {
    mc_rtc::log::error_and_throw("Failed to load RL policy: {}", e.what());
  }
  policyStepSize = config_("policies")[currentPolicyIndex]("policy_step_size", 0.01);

  int physicsStepSize = config_("policies")[currentPolicyIndex]("physics_step_size", 0.001);
  if(physicsStepSize - timeStep > 1e-6)
  {
    mc_rtc::log::warning("Physics step size ({:.3f} s) is larger than controller time step ({:.3f} s). This may cause "
                         "issues with the policy. Consider fixing the controller time step.",
                         physicsStepSize, timeStep);
  }
}

bool RLKinovaBallBalancingMcController::manageModeSwitching()
{
  if(controlModeChanged_)
  {
    if(isTorqueControl_)
    {
      mc_rtc::log::info("Switching to Torque Control");
      datastore().assign<std::string>("ControlMode", "Torque");
    }
    else
    {
      mc_rtc::log::info("Switching to Position Control");
      datastore().assign<std::string>("ControlMode", "Position");
    }
    controlModeChanged_ = false;
  }

  if(isTorqueControl_)
  {
    return mc_control::fsm::Controller::run(mc_solver::FeedbackType::ClosedLoopIntegrateReal);
  }
  else
  {
    return mc_control::fsm::Controller::run();
  }
}

void RLKinovaBallBalancingMcController::configureBallSpawn()
{
  if(!config_.has("ball_spawn"))
  {
    return;
  }
  auto c = config_("ball_spawn");
  // In-place overloads: every key is optional and leaves the member at its default when absent.
  c("enabled", spawnBallOnRLState);
  c("object", ballObjectName_);
  c("ball_radius", ballRadius_);
  c("clearance", ballSpawnClearance_);
  c("head_centre", ballHeadCentre_);
  c("head_semi_axes", ballHeadSemiAxes_);
}

bool RLKinovaBallBalancingMcController::simulationFtCompensationActive()
{
  if(!simFtCompensationEnabled_) { return false; }

  // mc_mujoco registers <robot>::SetPosW and <robot>::SetPDGains for every simulated robot in
  // MjSimImpl::makeDatastoreCalls, unconditionally. Neither exists on the real robot, and unlike
  // the <object>::SetPosW call spawnBall() keys off, these do not depend on anything being declared
  // in mc_mujoco.yaml - so this asks about the interface, not about the scene.
  //
  // A negative answer is deliberately never cached. This is first reached from the constructor, via
  // initializeRLPolicy(), and mc_mujoco only calls makeDatastoreCalls() *after* MCGlobalController
  // is built (mj_sim.cpp: 767 constructs, 771 registers, 791 init()). Latching "not simulation"
  // there would silently disable the compensation for the entire run - which is precisely the
  // failure this whole block exists to fix, so it must not be reintroduced by the detection itself.
  if(simDetected_ != 1 && datastore().has(robot().name() + "::SetPDGains"))
  {
    simDetected_ = 1;
    mc_rtc::log::info("[RLKinovaBallBalancingMcController] Simulated F/T sensor: applying the trained sensor "
                      "response (force gain {}{})",
                      simFtForceGain_, simFtRemoveCalibOffset_ ? ", calibration offset removed" : "");
  }
  return simDetected_ == 1;
}

sva::ForceVecd RLKinovaBallBalancingMcController::applySimulationFtResponse(const mc_rbdyn::ForceSensor & fs,
                                                                           const sva::ForceVecd & calibrated)
{
  if(!simulationFtCompensationActive()) { return calibrated; }

  // The policy reads what a *Bota* would report, because that is what it was trained on:
  // ball_balancing_mdp.py's ee_ft_wrench term runs MuJoCo's exact wrench through
  // apply_ft_sensor_response before compensating gravity. MuJoCo's own sensor is ideal, so in
  // simulation that model has to be added back here or the observation is not the one the policy
  // learned. On hardware it must NOT be: the sensor already behaves this way physically, and
  // applying it twice would be the same error with the opposite sign.
  //
  // Two corrections, and both have to happen on the raw reading, before gravity compensation -
  // that is where they happen on hardware, and the compensation is a subtraction, so correcting
  // the wrong side of it does not commute. Working backwards from what mc_rtc hands us:
  //
  //   calibrated = raw - wfToSensor            (ForceSensor::wrenchWithoutGravity)
  //   wfToSensor = offset + gravity            (ForceSensorCalibData::wfToSensor)
  //   wanted     = gain * raw - gravity
  //              = calibrated + offset - (1 - gain) * raw
  //
  // 1. The offset. It is a *measured* sensor bias, and MuJoCo's sensor has none for it to cancel,
  //    so subtracting it injects a constant error instead of removing one - here it is nearly 3 N,
  //    against a 0.64 N ball. The training term omits it for exactly this reason. Adding it back
  //    leaves gravity as the only thing compensated, which is what the policy saw.
  // 2. The force gain. The real sensor reads ~3.7 % low, and the calibration file was measured on
  //    hardware, so it holds 0.9633 * true_mass. Subtracting that measured gravity from an
  //    unscaled MuJoCo force over-subtracts by 3.7 % of the payload - about 0.11 N standing, plus
  //    3.7 % of the ball's own weight. The couple channel keeps unit gain: the residuals against
  //    the rigid model show no measurable error there, and the training model scales force only.
  // 3. The sensor frame. MuJoCo reports at the *model* sensor site, while mc_rtc's calibration
  //    resolves gravity in the *actual* sensor frame -- the two differ by the misalignment the
  //    calibration carries as X_f_ds (about 1 deg here). On hardware there is no such split: the
  //    Bota reports in its own physical frame, which is the actual one. The training term rotates
  //    MuJoCo's reading by the same X_f_ds for this reason, so the raw contribution below is
  //    rotated to match; left unrotated it leaves ~0.06 N on a 3 N reading, ~9 % of the ball.
  //
  // Writing R for MuJoCo's reading, A for that rotation, K for `calibrated` and W = fs.wrench():
  //
  //   wanted = gain * (A R) - gravity  =  K + offset - W + gain * (A W)
  //
  // which is what this returns (unit gain on the couple). With A = I it collapses to
  // K + offset - (1 - gain) * W, the form this had before the rotation was accounted for.
  sva::ForceVecd wrench = calibrated;
  if(simFtRemoveCalibOffset_) { wrench = wrench + fs.offset(); }
  const sva::ForceVecd & raw = fs.wrench();
  const Eigen::Matrix3d & R_f_ds = fs.calib().X_f_ds.rotation();
  return sva::ForceVecd(wrench.couple() - raw.couple() + R_f_ds * raw.couple(),
                        wrench.force() - raw.force() + simFtForceGain_ * (R_f_ds * raw.force()));
}

void RLKinovaBallBalancingMcController::spawnBall()
{
  // mc_mujoco registers <object>::SetPosW for every entry of the `objects` map in
  // ~/.config/mc_rtc/mc_mujoco/mc_mujoco.yaml (mj_sim.cpp: makeDatastoreCalls). The call is absent
  // on the real robot, and in simulation when the ball is simply not declared - neither is an
  // error here, so warn once and carry on rather than throwing from an FSM state entry.
  const auto setPosW = ballObjectName_ + "::SetPosW";
  if(!datastore().has(setPosW))
  {
    if(!ballSpawnUnavailableWarned_)
    {
      mc_rtc::log::warning("[RLKinovaBallBalancingMcController] No {} datastore call: not respawning the ball. Declare "
                           "the object in ~/.config/mc_rtc/mc_mujoco/mc_mujoco.yaml to enable it.",
                           setPosW);
      ballSpawnUnavailableWarned_ = true;
    }
    return;
  }

  // Where the ball goes: the highest point of the plate head, plus one ball radius and a small gap.
  //
  // The head is modelled as the ellipsoid {c + S u, ||u|| = 1} with c = ballHeadCentre_ and
  // S = diag(ballHeadSemiAxes_) in the control frame. The point of that ellipsoid furthest along a
  // direction d is c + S (S d)/||S d||. Computing it from the plate's current orientation, instead
  // of applying a fixed offset, keeps the drop point on top of the head whatever the tilt is - the
  // policy is free to be off the nominal posture when the state is entered.
  //
  // Poses come from the real robot: that is what MuJoCo actually simulates, and the ball is being
  // placed in MuJoCo's world.
  const auto & real_robot = realRobot(robots()[0].name());
  const sva::PTransformd & X_0_p = real_robot.bodyPosW(controlFrameName);
  // sva stores rotations as world -> body, hence E_0_p for world quantities and its transpose to
  // bring body quantities back to the world.
  const Eigen::Matrix3d & E_0_p = X_0_p.rotation();
  const Eigen::Vector3d up_p = E_0_p * Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d top_p =
      ballHeadCentre_ + ballHeadSemiAxes_.cwiseProduct(ballHeadSemiAxes_.cwiseProduct(up_p).normalized());

  Eigen::Vector3d ballPos = X_0_p.translation() + E_0_p.transpose() * top_p;
  ballPos.z() += ballRadius_ + ballSpawnClearance_;

  // Identity rotation: the ball is a sphere, and SetPosW zeroes its linear and angular velocities,
  // so this is a clean respawn rather than a teleport of a moving body.
  const sva::PTransformd X_0_ball(ballPos);
  datastore().call<void, const sva::PTransformd &>(setPosW, X_0_ball);
  mc_rtc::log::info("[RLKinovaBallBalancingMcController] Respawned '{}' at [{:.3f}, {:.3f}, {:.3f}]", ballObjectName_,
                    ballPos.x(), ballPos.y(), ballPos.z());
}

void RLKinovaBallBalancingMcController::wrenchTaskUpdate()
{
  auto & real_robot = realRobot(robots()[0].name());
  Eigen::VectorXd tau_d = Eigen::VectorXd::Zero(dofNumber);
  int i = 0;
  for(const auto & joint_name : jointNames_)
  {
    double q = real_robot.mbc().q[robot().jointIndexByName(joint_name)][0];
    double q_dot = real_robot.mbc().alpha[robot().jointIndexByName(joint_name)][0];
    tau_d(i) = kp_(i) * (q_rl(i) - q) - kd_(i) * q_dot;
    i++;
  }
  Eigen::VectorXd wrenchVec = wrenchTask->dynamicJacobianTranspose() * tau_d;
  wrenchTask_target = sva::ForceVecd(wrenchVec.head<3>(), wrenchVec.tail<3>());
}
