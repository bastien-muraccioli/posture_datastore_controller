#include "PostureDatastoreController_DefaultPosture.h"

#include "../PostureDatastoreController.h"

void PostureDatastoreController_DefaultPosture::configure(const mc_rtc::Configuration & config) {}

void PostureDatastoreController_DefaultPosture::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PostureDatastoreController &>(ctl_);
  if (!ctl.datastore().call<bool>("EF_Estimator::isActive")) {
    ctl.datastore().call("EF_Estimator::toggleActive");
  }

  size_t i = 0;
  for (const auto &j : ctl.robot().mb().joints()) {
      const std::string &joint_name = j.name();
      if(j.type() == rbd::Joint::Type::Rev)
      {
        if (const auto &t = ctl.posture_init_default[joint_name]; !t.empty()) {
            ctl.q_rl[i] = t[0];
            mc_rtc::log::info("[PostureDatastoreController] Joint {}: refPos {}", joint_name, ctl.q_rl[i]);
            i++;
        }
      }
  }

  ctl.endEffectorTarget_pt1 = ctl.endEffectorTarget_pt1_OOD;
  ctl.endEffectorTarget_pt2 = ctl.endEffectorTarget_pt2_OOD;
  ctl.endEffectorTarget_pt3 = ctl.endEffectorTarget_pt3_OOD;

  ctl.postureTask->reset();
  ctl.postureTask->stiffness(ctl.stiffnessMin);
  ctl.solver().addTask(ctl.postureTask);
}

bool PostureDatastoreController_DefaultPosture::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PostureDatastoreController &>(ctl_);
  
  if(isRobotStopped && !isReachedTarget)
  {
    ctl.stiffnessAdjustment();
    if(ctl.postureTask->eval().norm() < 0.05)
    {
      mc_rtc::log::info("[PostureDatastoreController_DefaultPosture] Target position reached, ready to change state");
      isReachedTarget = true;
      ctl.datastore().assign<std::string>("ControlMode", "Torque");
    }
  }

  if(!isRobotStopped)
  {
    ctl.stiffnessAdjustment();
    if(ctl.postureTask->eval().norm() < 0.05)
    {
      mc_rtc::log::info("[PostureDatastoreController_DefaultPosture] Robot is stopped, go back to the initial posture");
      isRobotStopped = true;

      ctl.postureTask->stiffness(ctl.stiffnessMin);
      ctl.postureTask->target(ctl.posture_init_default);
    }
  }
  // output("OK");
  return false;
}

void PostureDatastoreController_DefaultPosture::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PostureDatastoreController &>(ctl_);
  ctl.solver().removeTask(ctl.postureTask);
}

EXPORT_SINGLE_STATE("PostureDatastoreController_DefaultPosture", PostureDatastoreController_DefaultPosture)
