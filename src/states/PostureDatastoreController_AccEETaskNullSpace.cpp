#include "PostureDatastoreController_AccEETaskNullSpace.h"

#include "../PostureDatastoreController.h"

void PostureDatastoreController_AccEETaskNullSpace::configure(const mc_rtc::Configuration & config) {}

void PostureDatastoreController_AccEETaskNullSpace::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PostureDatastoreController &>(ctl_);
  ctl.datastore().assign<std::string>("ControlMode", "Torque");
  ctl.current_kp = ctl.kp_policy;
  ctl.current_kd = ctl.kd_policy;
  ctl.kp_value = ctl.current_kp[0];
  ctl.kd_value = ctl.current_kd[0];
  ctl.postureTask->stiffness(0.0);
  ctl.postureTask->damping(1.0);
  ctl.tasksComputation();
  // ctl.accEETask.reset();
  auto & realRobot = ctl.realRobot(ctl.robot().name());
  auto endEffectorTarget_pos = realRobot.mbc().bodyPosW[realRobot.bodyIndexByName(ctl.tool_frame)];
  ctl.accEETask->target(endEffectorTarget_pos);
  ctl.accEETask->refAccel(ctl.accEETask_target);
  Eigen::VectorXd zero_accel = Eigen::VectorXd::Zero(ctl.refAccel.size());
  ctl.postureTask->refAccel(zero_accel);
  ctl.postureTask->weight(1.0);
  ctl.solver().addTask(ctl.postureTask);
  ctl.solver().addTask(ctl.accEETask);
}

bool PostureDatastoreController_AccEETaskNullSpace::run(mc_control::fsm::Controller & ctl_)
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
  auto & realRobot = ctl.realRobot(ctl.robot().name());
  auto endEffectorTarget_pos = realRobot.mbc().bodyPosW[realRobot.bodyIndexByName(ctl.tool_frame)];
  ctl.accEETask->target(endEffectorTarget_pos);
  ctl.accEETask->refAccel(ctl.accEETask_target);
  
  // output("T_f");
  
  // output("OK");
  // return ctl.countPtReached();
  return false;
}

void PostureDatastoreController_AccEETaskNullSpace::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PostureDatastoreController &>(ctl_);
  ctl.solver().removeTask(ctl.postureTask);
  ctl.solver().removeTask(ctl.accEETask);
  ctl.cleanState();
}

EXPORT_SINGLE_STATE("PostureDatastoreController_AccEETaskNullSpace", PostureDatastoreController_AccEETaskNullSpace)
