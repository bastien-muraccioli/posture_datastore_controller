#include "PostureDatastoreController_PureRL.h"

#include "../PostureDatastoreController.h"

void PostureDatastoreController_PureRL::configure(const mc_rtc::Configuration & config) {}

void PostureDatastoreController_PureRL::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PostureDatastoreController &>(ctl_);
  ctl.datastore().assign<std::string>("ControlMode", "Torque");
  ctl.kp_vector = ctl.kp_torque_vector;
  ctl.kd_vector = ctl.kd_torque_vector;
  ctl.kp_value = ctl.kp_vector[0];
  ctl.kd_value = ctl.kd_vector[0];
  ctl.isPureRL = true;
  ctl.compPostureTask->stiffness(0.0);
  ctl.compPostureTask->damping(0.0);
  ctl.counter = 0.0; // Reset the counter for posture updates
  ctl.refPos_last = ctl.refPos; // Store the initial reference position
}

bool PostureDatastoreController_PureRL::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PostureDatastoreController &>(ctl_);
                      
  if(ctl.datastore().has("ros_posture_pub_sub"))
  {
    auto posture = ctl.datastore().get<std::map<std::string, std::vector<double>>>("ros_posture_pub_sub");
    if (posture.size() > 0)
    {
      // ctl.compPostureTask->target(posture);
      // convert the posture to the ctl.refPos
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
  return false;
}

void PostureDatastoreController_PureRL::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PostureDatastoreController &>(ctl_);
  ctl.isPureRL = false;
}

EXPORT_SINGLE_STATE("PostureDatastoreController_PureRL", PostureDatastoreController_PureRL)
