#include "RLKinovaBallBalancingMcController_RL.h"

#include "../RLKinovaBallBalancingMcController.h"

void RLKinovaBallBalancingMcController_RL::configure(const mc_rtc::Configuration & config) {}

void RLKinovaBallBalancingMcController_RL::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RLKinovaBallBalancingMcController &>(ctl_);
  ctl.utilsClass.start_rl_state(ctl, "RL_State");
  ctl.useQP = true; // Force the QP to be used in this state, the mirror of RLWithoutQP
  ctl.solver().addTask(ctl.torqueJointTask);
}

bool RLKinovaBallBalancingMcController_RL::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RLKinovaBallBalancingMcController &>(ctl_);
  ctl.utilsClass.run_rl_state(ctl);
  ctl.torqueJointTask->setPosTarget(ctl.q_rl);
  return false;
}

void RLKinovaBallBalancingMcController_RL::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RLKinovaBallBalancingMcController &>(ctl_);
  ctl.utilsClass.teardown_rl_state(ctl);
  ctl.solver().removeTask(ctl.torqueJointTask);
}

EXPORT_SINGLE_STATE("RLKinovaBallBalancingMcController_RL", RLKinovaBallBalancingMcController_RL)
