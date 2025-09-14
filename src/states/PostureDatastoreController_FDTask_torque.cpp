#include "PostureDatastoreController_FDTask_torque.h"

#include "../PostureDatastoreController.h"

void PostureDatastoreController_FDTask_torque::configure(const mc_rtc::Configuration & config) {}

void PostureDatastoreController_FDTask_torque::start(mc_control::fsm::Controller & ctl_)
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
  ctl.postureTask->refAccel(ctl.refAccel);
  ctl.solver().addTask(ctl.postureTask);
}

bool PostureDatastoreController_FDTask_torque::run(mc_control::fsm::Controller & ctl_)
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
  
  output("T_f");
  
  // output("OK");
  return ctl.countPtReached();
}

void PostureDatastoreController_FDTask_torque::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PostureDatastoreController &>(ctl_);
  ctl.solver().removeTask(ctl.postureTask);
  ctl.cleanState();
}

EXPORT_SINGLE_STATE("PostureDatastoreController_FDTask_torque", PostureDatastoreController_FDTask_torque)
