#include "PostureDatastoreController_TorqueTask_pos.h"

#include "../PostureDatastoreController.h"

void PostureDatastoreController_TorqueTask_pos::configure(const mc_rtc::Configuration & config) {}

void PostureDatastoreController_TorqueTask_pos::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PostureDatastoreController &>(ctl_);
  ctl.datastore().assign<std::string>("ControlMode", "Position");
  ctl.kp_vector = ctl.kp_position_vector;
  ctl.kd_vector = ctl.kd_position_vector;
  ctl.kp_value = ctl.kp_vector[0];
  ctl.kd_value = ctl.kd_vector[0];
  ctl.isRLQP = true;
  ctl.compPostureTask->stiffness(0.0);
  ctl.compPostureTask->damping(0.0);
  ctl.FDTask();
  ctl.torqueTask->target(ctl.torque);
  ctl.solver().addTask(ctl.torqueTask);
}

bool PostureDatastoreController_TorqueTask_pos::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PostureDatastoreController &>(ctl_);
                      
  if(ctl.datastore().has("ros_posture_pub_sub"))
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
              ctl.refPos[i] = t[0];
              i++;
          }
        }
      }
    }
  }
  ctl.FDTask();  
  ctl.torqueTask->target(ctl.torque);
  // output("OK");
  return false;
}

void PostureDatastoreController_TorqueTask_pos::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PostureDatastoreController &>(ctl_);
  ctl.isRLQP = false;
}



EXPORT_SINGLE_STATE("PostureDatastoreController_TorqueTask_pos", PostureDatastoreController_TorqueTask_pos)
