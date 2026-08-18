#include "utils.h"
#include <Eigen/src/Core/Matrix.h>
#include <mc_rtc/logging.h>
#include <SpaceVecAlg/SpaceVecAlg>

#include "RLKinovaBallBalancingMcController.h"

void utils::start_rl_state(mc_control::fsm::Controller & ctl_, std::string state_name)
{
  auto & ctl = static_cast<RLKinovaBallBalancingMcController&>(ctl_);
  state_name_ = state_name;
  mc_rtc::log::info("{} state started", state_name);

  syncTime_ = ctl.policyStepSize;

  ctl.initializeRLObservation();

  // Entering an RL state means starting a balancing attempt: put a ball back on the plate, wherever
  // the previous one ended up. No-op outside mc_mujoco.
  if(ctl.spawnBallOnRLState)
  {
    ctl.spawnBall();
  }

  if(!ctl.rlPolicy || !ctl.rlPolicy->isLoaded())
  {
    mc_rtc::log::error("RL policy not loaded in {} state", state_name);
    return;
  }

  ctl.gui()->addElement(
    {"RLKinovaBallBalancingMcController", state_name},
    mc_rtc::gui::Label("Policy Loaded", [&ctl]() { 
      return ctl.rlPolicy->isLoaded() ? "Yes" : "No"; 
    }),
    mc_rtc::gui::Label("Observation Size", [&ctl]() { 
      return std::to_string(ctl.rlPolicy->getObservationSize()); 
    }),
    mc_rtc::gui::Label("Action Size", [&ctl]() { 
      return std::to_string(ctl.rlPolicy->getActionSize()); 
    })
  );

  mc_rtc::log::success("{} state initialization completed", state_name);
}

void utils::run_rl_state(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RLKinovaBallBalancingMcController&>(ctl_);
  try
  {
    syncTime_ += ctl.timeStep;
    if(syncTime_ >= ctl.policyStepSize)
    {
      ctl.currentObservation = getCurrentObservation(ctl);
      ctl.currentAction = ctl.rlPolicy->predict(ctl.currentObservation);
      ctl.currentActionScaled = ctl.actionScale.cwiseProduct(ctl.currentAction);
      // Run new inference and update target position, scaled by action scale
      ctl.q_rl = ctl.q_zero + ctl.currentActionScaled;
      syncTime_ = 0.0;
    }
  }
  catch(const std::exception & e)
  {
    mc_rtc::log::error("Error during RL state run: {}", e.what());
  }
}

void utils::teardown_rl_state(mc_control::fsm::Controller & ctl_)
{
  ctl_.gui()->removeCategory({"RLKinovaBallBalancingMcController", state_name_});
}

Eigen::VectorXd utils::getCurrentObservation(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RLKinovaBallBalancingMcController&>(ctl_);
  Eigen::VectorXd obs = Eigen::VectorXd::Zero(ctl.rlPolicy->getObservationSize());

  int offset = 0;
  auto append = [&](const Eigen::VectorXd& v) {
    obs.segment(offset, v.size()) = v;
    offset += v.size();
  };

  switch (ctl.currentPolicyIndex) {
    case 0:
    {
      for (int i = ctl.HISTORY_SIZE - 1; i > 0; --i) {
          ctl.jointVel[i] = ctl.jointVel[i - 1];
          ctl.jointPos[i] = ctl.jointPos[i - 1];
          ctl.jointAction[i] = ctl.jointAction[i - 1];
          ctl.eePos[i] = ctl.eePos[i - 1];
          ctl.eeQuat[i] = ctl.eeQuat[i - 1];
          ctl.eeLinVel[i] = ctl.eeLinVel[i - 1];
          ctl.eeAngVel[i] = ctl.eeAngVel[i - 1];
          ctl.eeWrench[i] = ctl.eeWrench[i - 1];
      }
      ctl.initializeRLObservation();
      for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) append(ctl.jointPos[i]);
      for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) append(ctl.jointVel[i]);
      for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) append(ctl.eePos[i]);
      for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) append(ctl.eeQuat[i]);
      for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) append(ctl.eeLinVel[i]);
      for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) append(ctl.eeAngVel[i]);
      for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) append(ctl.eeWrench[i]);
      for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) append(ctl.jointAction[i]);

      break;
    }
    default:
    {
      mc_rtc::log::error("Unknown policy index: {}", ctl.currentPolicyIndex);
      break;
    }
  }
  
  return obs;
}
