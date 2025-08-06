#include "PostureDatastoreController_Initial.h"

#include "../PostureDatastoreController.h"

void PostureDatastoreController_Initial::configure(const mc_rtc::Configuration & config) {}

void PostureDatastoreController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PostureDatastoreController &>(ctl_);
  ctl.compPostureTask->reset();
  ctl.compPostureTask->stiffness(ctl.stiffnessMin);
  ctl.solver().addTask(ctl.compPostureTask);
}

bool PostureDatastoreController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PostureDatastoreController &>(ctl_);
  
  if(isRobotStopped && !isReachedTarget)
  {
    ctl.stiffnessAdjustment();
    if(ctl.compPostureTask->eval().norm() < 0.05)
    {
      mc_rtc::log::info("[PostureDatastoreController_Initial] Target position reached, ready to change state");
      isReachedTarget = true;
      ctl.datastore().assign<std::string>("ControlMode", "Torque");
    }
  }

  if(!isRobotStopped)
  {
    ctl.stiffnessAdjustment();
    if(ctl.compPostureTask->eval().norm() < 0.05)
    {
      mc_rtc::log::info("[PostureDatastoreController_Initial] Robot is stopped, go back to the initial posture");
      isRobotStopped = true;
      ctl.datastore().assign<std::string>("ControlMode", "Torque");
      ctl.compPostureTask->stiffness(ctl.stiffnessMin);
      ctl.compPostureTask->target(ctl.posture);
    }
  }
  // output("OK");
  return false;
}

void PostureDatastoreController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PostureDatastoreController &>(ctl_);
  ctl.solver().removeTask(ctl.compPostureTask);
}

EXPORT_SINGLE_STATE("PostureDatastoreController_Initial", PostureDatastoreController_Initial)
