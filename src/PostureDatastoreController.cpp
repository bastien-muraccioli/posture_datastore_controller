#include "PostureDatastoreController.h"

PostureDatastoreController::PostureDatastoreController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{

  // Initialize the constraints
  selfCollisionConstraint->setCollisionsDampers(solver(), {1.8, 70.0});
  solver().removeConstraintSet(dynamicsConstraint);
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
    new mc_solver::DynamicsConstraint(robots(), 0, {0.1, 0.01, 0.0, 1.8, 70.0}, 0.9, true));
  solver().addConstraintSet(dynamicsConstraint);

  posture = {{"joint_1", {0}}, {"joint_2", {0.262}}, {"joint_3", {3.14}}, {"joint_4", {-2.269}},
                   {"joint_5", {0}}, {"joint_6", {0.96}},  {"joint_7", {1.57}}};
  
  // Remove the default posture task created by the FSM
  solver().removeTask(getPostureTask(robot().name()));

  compPostureTask = std::make_shared<mc_tasks::CompliantPostureTask>(solver(), robot().robotIndex(), 1, 1);
  compPostureTask->reset();
  compPostureTask->target(posture);
  compPostureTask->stiffness(10.0);
  solver().addTask(compPostureTask);

  // Kinova Gen3 datastore
  datastore().make<std::string>("ControlMode", "Position");
  datastore().make<std::string>("TorqueMode", "Custom");
  datastore().make_call("getPostureTask", [this]() -> mc_tasks::PostureTaskPtr { return compPostureTask; });

  mc_rtc::log::success("PostureDatastoreController init done ");
}

bool PostureDatastoreController::run()
{
  auto ctrl_mode = datastore().get<std::string>("ControlMode");
  if(ctrl_mode.compare("Position") == 0)
  {
    return mc_control::fsm::Controller::run(mc_solver::FeedbackType::OpenLoop);
  }
  else
  {
    return mc_control::fsm::Controller::run(mc_solver::FeedbackType::ClosedLoopIntegrateReal);
  }
}

void PostureDatastoreController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}
