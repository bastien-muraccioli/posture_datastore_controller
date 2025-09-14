#include "PostureDatastoreController_Initial.h"

#include "../PostureDatastoreController.h"

void PostureDatastoreController_Initial::configure(const mc_rtc::Configuration & config) {}

void PostureDatastoreController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PostureDatastoreController &>(ctl_);
  if (!ctl.datastore().call<bool>("EF_Estimator::isActive")) {
    ctl.datastore().call("EF_Estimator::toggleActive");
  }
  ctl.postureTask->reset();
  ctl.postureTask->stiffness(ctl.stiffnessMin);
  ctl.solver().addTask(ctl.postureTask);
}

bool PostureDatastoreController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PostureDatastoreController &>(ctl_);
  
  if(isRobotStopped && !isReachedTarget)
  {
    ctl.stiffnessAdjustment();
    if(ctl.postureTask->eval().norm() < 0.05)
    {
      mc_rtc::log::info("[PostureDatastoreController_Initial] Target position reached, ready to change state");
      isReachedTarget = true;
      ctl.datastore().assign<std::string>("ControlMode", "Torque");
    }
  }

  if(!isRobotStopped)
  {
    ctl.stiffnessAdjustment();
    if(ctl.postureTask->eval().norm() < 0.05)
    {
      mc_rtc::log::info("[PostureDatastoreController_Initial] Robot is stopped, go back to the initial posture");
      isRobotStopped = true;
      ctl.postureTask->stiffness(ctl.stiffnessMin);
      ctl.postureTask->target(ctl.posture_init_rl);
    }
  }
  // output("OK");
  return false;
}

void PostureDatastoreController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PostureDatastoreController &>(ctl_);
  ctl.solver().removeTask(ctl.postureTask);
}

EXPORT_SINGLE_STATE("PostureDatastoreController_Initial", PostureDatastoreController_Initial)
