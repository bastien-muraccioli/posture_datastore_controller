#include "PostureDatastoreController_PureRL.h"

#include "../PostureDatastoreController.h"

void PostureDatastoreController_PureRL::configure(const mc_rtc::Configuration & config) {}

void PostureDatastoreController_PureRL::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PostureDatastoreController &>(ctl_);
  ctl.datastore().assign<std::string>("ControlMode", "Torque");
  ctl.current_kp = ctl.kp_policy;
  ctl.current_kd = ctl.kd_policy;
  ctl.kp_value = ctl.current_kp[0];
  ctl.kd_value = ctl.current_kd[0];
  ctl.isPureRL = true;
  ctl.counter = 0.0; // Reset the counter for posture updates
  ctl.q_rl_last = ctl.q_rl; // Store the initial reference position
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
              ctl.q_rl[i] = t[0];
              i++;
          }
        }
      }
    }
  }
  ctl.tasksComputation();
  
  output("FD_f");

  return ctl.countPtReached();
}

void PostureDatastoreController_PureRL::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PostureDatastoreController &>(ctl_);
  ctl.isPureRL = false;
  ctl.cleanState();
}

EXPORT_SINGLE_STATE("PostureDatastoreController_PureRL", PostureDatastoreController_PureRL)
