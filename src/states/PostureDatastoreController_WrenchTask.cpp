#include "PostureDatastoreController_WrenchTask.h"

#include "../PostureDatastoreController.h"

void PostureDatastoreController_WrenchTask::configure(const mc_rtc::Configuration & config) {}

void PostureDatastoreController_WrenchTask::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PostureDatastoreController &>(ctl_);
  ctl.datastore().assign<std::string>("ControlMode", "Torque");
  ctl.current_kp = ctl.kp_policy;
  ctl.current_kd = ctl.kd_policy;
  ctl.kp_value = ctl.current_kp[0];
  ctl.kd_value = ctl.current_kd[0];
  ctl.postureTask->stiffness(0.0);
  ctl.postureTask->damping(0.0);
  ctl.tasksComputation();
  ctl.wrenchTask->target(ctl.wrenchTask_target);
  ctl.accEETask->weight(1000.0);
  ctl.postureTask->refAccel(ctl.refAccel);
  ctl.postureTask->weight(1.0);
  ctl.solver().addTask(ctl.postureTask);
  ctl.solver().addTask(ctl.wrenchTask);
}

bool PostureDatastoreController_WrenchTask::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PostureDatastoreController &>(ctl_);
                      
  if (ctl.datastore().has("ros_posture_pub_sub"))
  {
    auto posture = ctl.datastore().get<std::map<std::string, std::vector<double>>>("ros_posture_pub_sub");
    if (posture.size() > 0)
    {
      size_t i = 0;
      for (const auto &j : ctl.robot().mb().joints()) {
        const std::string &joint_name = j.name();
        if(j.type() == rbd::Joint::Type::Rev)
        {
          if (const auto &t = posture[joint_name]; !t.empty()) {
              ctl.q_rl[i] = t[0];
              i++;
          }
        }
      }
    }
  }
  ctl.tasksComputation();  
  ctl.postureTask->refAccel(ctl.refAccel);
  ctl.wrenchTask->target(ctl.wrenchTask_target);
  return false;
}

void PostureDatastoreController_WrenchTask::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PostureDatastoreController &>(ctl_);
  ctl.solver().removeTask(ctl.postureTask);
  ctl.solver().removeTask(ctl.wrenchTask);
  ctl.cleanState();
}

EXPORT_SINGLE_STATE("PostureDatastoreController_WrenchTask", PostureDatastoreController_WrenchTask)
