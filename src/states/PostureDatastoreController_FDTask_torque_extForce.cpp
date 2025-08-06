#include "PostureDatastoreController_FDTask_torque_extForce.h"

#include "../PostureDatastoreController.h"

void PostureDatastoreController_FDTask_torque_extForce::configure(const mc_rtc::Configuration & config) {}

void PostureDatastoreController_FDTask_torque_extForce::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PostureDatastoreController &>(ctl_);
  ctl.datastore().assign<std::string>("ControlMode", "Torque");
  if (!ctl.datastore().call<bool>("EF_Estimator::isActive")) {
    ctl.datastore().call("EF_Estimator::toggleActive");
  }
  ctl.kp_vector = ctl.kp_torque_vector;
  ctl.kd_vector = ctl.kd_torque_vector;
  ctl.kp_value = ctl.kp_vector[0];
  ctl.kd_value = ctl.kd_vector[0];
  ctl.compPostureTask->stiffness(0.0);
  ctl.compPostureTask->damping(0.0);
  ctl.compensateExternalForces = true; // Enable external force compensation
  ctl.FDTask();
  ctl.compPostureTask->refAccel(ctl.refAccel);
  ctl.solver().addTask(ctl.compPostureTask);
}

bool PostureDatastoreController_FDTask_torque_extForce::run(mc_control::fsm::Controller & ctl_)
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
              ctl.refPos[i] = t[0];
              i++;
          }
        }
      }
    }
  }
  ctl.FDTask();  
  ctl.compPostureTask->refAccel(ctl.refAccel);
  
  // output("OK");
  return false;
}

void PostureDatastoreController_FDTask_torque_extForce::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PostureDatastoreController &>(ctl_);
  ctl.solver().removeTask(ctl.compPostureTask);
  if (ctl.datastore().call<bool>("EF_Estimator::isActive")) {
    ctl.datastore().call("EF_Estimator::toggleActive");
  }
  ctl.compensateExternalForces = false;
}

EXPORT_SINGLE_STATE("PostureDatastoreController_FDTask_torque_extForce", PostureDatastoreController_FDTask_torque_extForce)
