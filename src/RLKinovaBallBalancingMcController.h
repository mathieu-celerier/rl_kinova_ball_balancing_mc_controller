#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_rbdyn/ForceSensor.h>
#include <mc_tasks/TorqueJointTask.h>

#include <mc_tasks/CompliantPostureTask.h>
#include <mc_tasks/WrenchTask.h>

#include "api.h"

#include "RLPolicyInterface.h"
#include "utils.h"
#include <string>
#include <vector>

struct RLKinovaBallBalancingMcController_DLLAPI RLKinovaBallBalancingMcController : public mc_control::fsm::Controller
{
  RLKinovaBallBalancingMcController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;
  void reset(const mc_control::ControllerResetData & reset_data) override;
  void initializeRLObservation();
  void wrenchTaskUpdate();

  // Drop the simulated ball back onto the plate. Called on entry of every RL state (utils.cpp) and
  // from the GUI. Silently does nothing outside mc_mujoco - see the definition.
  void spawnBall();
  // Whether entering an RL state respawns the ball. Toggled from the GUI.
  bool spawnBallOnRLState = true;

  // Task
  std::shared_ptr<mc_tasks::TorqueJointTask> torqueJointTask;
  std::shared_ptr<mc_tasks::CompliantPostureTask> compliantPostureTask;
  const std::map<std::string, std::vector<double>> posture_init_default = {
      {"joint_1", {0}}, {"joint_2", {0.262}}, {"joint_3", {3.14}}, {"joint_4", {-2.269}},
      {"joint_5", {0}}, {"joint_6", {0.96}},  {"joint_7", {1.57}}};
  std::shared_ptr<mc_tasks::WrenchTask> wrenchTask;
  sva::ForceVecd wrenchTask_target;

  int dofNumber = 0;

  // Public RL related variables
  Eigen::VectorXd q_rl;
  Eigen::VectorXd q_zero; // Reference joint positions

  Eigen::VectorXd currentObservation;
  Eigen::VectorXd currentAction; // Raw output from the policy
  Eigen::VectorXd
      currentActionScaled; // Scaled action after applying actionScale, share the same size as q_rl, for the joints that
                           // are not controlled by the policy the value is 0 in currentActionScaled.

  Eigen::VectorXd actionScale;
  double policyStepSize; // Time interval between policy updates in seconds

  size_t currentPolicyIndex = 0;
  std::unique_ptr<RLPolicyInterface> rlPolicy;
  utils utilsClass; // Utility functions for RL controller

  // Body/frame the WrenchTask and the end-effector observations are expressed in. Must match the
  // frame the policy was trained in, so it is read from the controller configuration
  // ("control_frame") rather than hardcoded -- policies trained against a different end-effector
  // may well name a different link.
  std::string controlFrameName = "racquet_frame";

  bool useQP = true;

  // observation
  static constexpr int HISTORY_SIZE = 5; // Number of past time steps to include in the observation
  std::array<Eigen::VectorXd, HISTORY_SIZE> jointPos, jointVel, jointAction;
  std::array<Eigen::Vector3d, HISTORY_SIZE> eePos, eeLinVel, eeAngVel;
  std::array<Eigen::Vector4d, HISTORY_SIZE> eeQuat;
  std::array<Eigen::Vector6d, HISTORY_SIZE> eeWrench;

private:
  mc_rtc::Configuration config_;
  void addLog();
  void addGui();

  void initializeRobot();
  void configRL();
  void initializeRLPolicy();
  void switchPolicy(int policyIndex); // Switch to a different policy at runtime

  bool manageModeSwitching(); // Handle switching between Torque and Position control modes
  bool byPassQPControl(); // Directly use RL output without QP modifications

  void configureBallSpawn(); // Read the "ball_spawn" configuration block

  // Simulation-only F/T sensor compensation, all read from the "sim_ft_sensor" configuration
  // block. MuJoCo's force sensor is ideal; the real Bota is not, and the policy was trained on a
  // model of the real one. Two corrections, both no-ops on hardware - see the definitions.
  bool simulationFtCompensationActive();
  sva::ForceVecd applySimulationFtResponse(const mc_rbdyn::ForceSensor & fs, const sva::ForceVecd & calibrated);
  bool simFtCompensationEnabled_ = true;
  double simFtForceGain_ = 0.9633;
  bool simFtRemoveCalibOffset_ = true;
  // -1 = not yet determined. Resolved on first use rather than at construction: the simulation
  // registers the datastore calls this keys off after the controller is built.
  int simDetected_ = -1;
  bool simFtCompensationLogged_ = false;

  // Ball respawn, all read from the "ball_spawn" configuration block. The head of the plate is
  // modelled as an ellipsoid in the control frame: defaults measured off mc_kinova's plate.STL,
  // whose ovoid spans [-0.2284, -0.0233] along the mesh y axis with a maximum radius of 0.0615 m,
  // mapped into the link frame by the visual origin rpy="-pi/2 0 pi/4" (mesh -y becomes link +z).
  std::string ballObjectName_ = "ball";
  double ballRadius_ = 0.0335; // kinova_ball.xml sphere size
  double ballSpawnClearance_ = 0.01; // Gap left between plate and ball, so it settles by falling
  Eigen::Vector3d ballHeadCentre_ = {0.0, 0.0, 0.1257};
  Eigen::Vector3d ballHeadSemiAxes_ = {0.0615, 0.0615, 0.1025};
  bool ballSpawnUnavailableWarned_ = false;

  std::string robotName_;
  std::vector<std::string> jointNames_;

  // Mode switching
  bool isTorqueControl_ = false;
  bool controlModeChanged_ = false;

  // Constraint configuration
  double velPercent_ = 0.95;
  double dsPercent_ = 0.01;
  double diPercent_ = 0.1;

  // Gains
  double pdGainsRatio_ = 1.0;
  Eigen::VectorXd kp_; // Gains set to the robot/simulator = pd_gains_ratio * kp_base
  Eigen::VectorXd kd_; // Gains set to the robot/simulator = pd_gains_ratio * kd_base
  Eigen::VectorXd kpBase_; // Base RL PD gains from config
  Eigen::VectorXd kdBase_; // Base RL PD gains from config

  Eigen::VectorXd tau_rl_; // Torque output from the RL policy

  // RL
  std::vector<std::string> policyPaths_;
};
