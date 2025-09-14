#include "PostureDatastoreController.h"
#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/NumberInput.h>
#include <mc_rtc/gui/Rotation.h>
#include <string>

PostureDatastoreController::PostureDatastoreController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{

  tool_frame = config("tool_frame", (std::string) "FT_adapter");

  // Initialize the constraints
  selfCollisionConstraint->setCollisionsDampers(solver(), {1.8, 70.0});
  solver().removeConstraintSet(dynamicsConstraint);
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
    new mc_solver::DynamicsConstraint(robots(), 0, {0.1, 0.01, 0.0, 1.8, 70.0}, 0.9, true));
  solver().addConstraintSet(dynamicsConstraint);

  // Default posture target
  posture_init_default = {{"joint_1", {0}}, {"joint_2", {0.262}}, {"joint_3", {3.14}}, {"joint_4", {-2.269}},
                   {"joint_5", {0}}, {"joint_6", {0.96}},  {"joint_7", {1.57}}};

  // posture during the training
  posture_init_rl = {{"joint_1", {0.0}}, {"joint_2", {0.65}}, {"joint_3", {0.0}}, {"joint_4", {1.89}},
                   {"joint_5", {0.0}}, {"joint_6", {0.6}},  {"joint_7", {-1.57}}};

  torque_target = {{"joint_1", {0.0}}, {"joint_2", {0.0}}, {"joint_3", {0.0}}, {"joint_4", {0.0}},
                   {"joint_5", {0.0}}, {"joint_6", {0.0}},  {"joint_7", {0.0}}};

  endEffectorTarget_pt1 = Eigen::Vector3d(0.5, 0.0, 0.2);
  endEffectorTarget_pt2 = Eigen::Vector3d(0.4, -0.15, 0.3);
  endEffectorTarget_pt3 = Eigen::Vector3d(0.6, 0.1, 0.45);

  endEffectorTarget_pt1_OOD = Eigen::Vector3d(0.4, 0.1, 0.35);
  endEffectorTarget_pt2_OOD = Eigen::Vector3d(0.35, -0.2, 0.5);
  endEffectorTarget_pt3_OOD = Eigen::Vector3d(0.3, 0.2, 0.45);

  // endEffectorTarget_pt1_biased_by_posture

  endEffectorTarget_pos = robot().mbc().bodyPosW[robot().bodyIndexByName(tool_frame)].translation();

  distance_pt1 = endEffectorTarget_pt1 - endEffectorTarget_pos;
  distance_pt2 = endEffectorTarget_pt2 - endEffectorTarget_pos;
  distance_pt3 = endEffectorTarget_pt3 - endEffectorTarget_pos;

  distance_pt1_norm = distance_pt1.norm();
  distance_pt2_norm = distance_pt2.norm();
  distance_pt3_norm = distance_pt3.norm();

  auto orientation = Eigen::Quaterniond(0.7071, 0.0, 0.7071, 0.0).normalized().toRotationMatrix();

  dofNumber = robot().mb().nrDof();
  mc_rtc::log::info("[PostureDatastoreController] Robot has {} DoF", dofNumber);
  refAccel = Eigen::VectorXd::Zero(dofNumber);
  q_rl = Eigen::VectorXd::Zero(dofNumber);
  q_rl_last = Eigen::VectorXd::Zero(dofNumber); // Initialize last reference position
  tau_d = Eigen::VectorXd::Zero(dofNumber);
  currentPos = Eigen::VectorXd::Zero(dofNumber);
  currentVel = Eigen::VectorXd::Zero(dofNumber);
  current_kp = Eigen::VectorXd::Zero(dofNumber);
  current_kd = Eigen::VectorXd::Zero(dofNumber);
  kp_robot = Eigen::VectorXd::Zero(dofNumber);
  kd_robot = Eigen::VectorXd::Zero(dofNumber);
  kp_policy = Eigen::VectorXd::Zero(dofNumber);
  kd_policy = Eigen::VectorXd::Zero(dofNumber);
  ddot_qp = Eigen::VectorXd::Zero(dofNumber); // Desired acceleration in the QP solver
  q_cmd = Eigen::VectorXd::Zero(dofNumber); // The commended position send to the internal PD of the robot
  tau_cmd = Eigen::VectorXd::Zero(dofNumber); // The commended position after PD control

  // Compute A and C for stiffness adjustment
  A = (-stiffnessMax + stiffnessMin) / (exp(k_slope) - 1);
  C = stiffnessMax - A;
  
  std::map<std::string, double> kp_robot_vector = config("kp_robot");
  std::map<std::string, double> kd_robot_vector = config("kd_robot");
  std::map<std::string, double> kp_policy_vector = config("kp_policy");
  std::map<std::string, double> kd_policy_vector = config("kd_policy");

  size_t i = 0;
  for (const auto &j : robot().mb().joints()) {
      const std::string &joint_name = j.name();
      if(j.type() == rbd::Joint::Type::Rev)
      {
        jointNames.emplace_back(joint_name);
        if (const auto &t = posture_init_rl[joint_name]; !t.empty()) {
            current_kp[i] = kp_robot_vector.at(joint_name);
            current_kd[i] = kd_robot_vector.at(joint_name);
            kp_robot[i] = kp_robot_vector.at(joint_name);
            kd_robot[i] = kd_robot_vector.at(joint_name);
            kp_policy[i] = kp_policy_vector.at(joint_name);
            kd_policy[i] = kd_policy_vector.at(joint_name);
            q_rl[i] = t[0];
            mc_rtc::log::info("[PostureDatastoreController] Joint {}: refPos {}, kp {}, kd {}", joint_name, q_rl[i], current_kp[i], current_kd[i]);
            i++;
        }
      }
  }

  kp_value = current_kp[0]; // Assuming all joints have the same kp value
  kd_value = current_kd[0]; // Assuming all joints have the same kd value
  
  // Remove the default posture task created by the FSM
  solver().removeTask(getPostureTask(robot().name()));

  postureTask = std::make_shared<mc_tasks::PostureTask>(solver(), robot().robotIndex(), 1, 1);
  postureTask->target(posture_init_rl);
  // compPostureTask->stiffness(stiffnessMin);
  // solver().addTask(compPostureTask);

  torqueTask = std::make_shared<mc_tasks::TorqueTask>(solver(), robot().robotIndex());

  logger().addLogEntry("RLController_refAccel", [this]() { return refAccel; });
  logger().addLogEntry("RLController_q_rl", [this]() { return q_rl; });
  logger().addLogEntry("RLController_tau_d", [this]() { return tau_d; });
  logger().addLogEntry("RLController_kp", [this]() { return current_kp; });
  logger().addLogEntry("RLController_kd", [this]() { return current_kd; });
  logger().addLogEntry("RLController_currentPos", [this]() { return currentPos; });
  logger().addLogEntry("RLController_currentVel", [this]() { return currentVel; });
  logger().addLogEntry("RLController_q_cmd", [this]() { return q_cmd; });
  logger().addLogEntry("RLController_ddot_qp", [this]() { return ddot_qp; });
  logger().addLogEntry("RLController_tau_cmd", [this]() { return tau_cmd; });
  logger().addLogEntry("RLController_distance_pt1", [this]() { return distance_pt1; });
  logger().addLogEntry("RLController_distance_pt2", [this]() { return distance_pt2; });
  logger().addLogEntry("RLController_distance_pt3", [this]() { return distance_pt3; });
  logger().addLogEntry("RLController_distance_pt1_norm", [this]() { return distance_pt1_norm; });
  logger().addLogEntry("RLController_distance_pt2_norm", [this]() { return distance_pt2_norm; });
  logger().addLogEntry("RLController_distance_pt3_norm", [this]() { return distance_pt3_norm; });
  logger().addLogEntry("RLController_endEffectorTarget_pos", [this]() { return endEffectorTarget_pos; });

  logger().addLogEntry("RLController_convergedToPtTarget", [this]() { return convergedToPtTarget; });
  logger().addLogEntry("RLController_convergedToPtInit", [this]() { return convergedToPtInit; });

  // Kinova Gen3 datastore
  datastore().make<std::string>("ControlMode", "Torque");
  datastore().make<std::string>("TorqueMode", "Custom");
  datastore().make_call("getPostureTask", [this]() -> mc_tasks::PostureTaskPtr { return postureTask; });

  // Add GUI to modify kp and kd
  gui()->addElement({"PostureDatastoreController"},
    mc_rtc::gui::ArrayInput("Kp Vector",
      [this]() { return current_kp; },
      [this](const Eigen::VectorXd & v) { current_kp = v; }),
    mc_rtc::gui::ArrayInput("Kd Vector",
      [this]() { return current_kd; },
      [this](const Eigen::VectorXd & v) { current_kd = v; }),
    mc_rtc::gui::NumberInput("Kp",
      [this]() { return kp_value; },
      [this](const double v) {
        kp_value = v;
        for(size_t i = 0; i < current_kp.size(); ++i)
        {
          current_kp[i] = kp_value;
        }
      }),
    mc_rtc::gui::NumberInput("Kd",
      [this]() { return kd_value; },
      [this](const double v) {
        kd_value = v;
        for(size_t i = 0; i < current_kd.size(); ++i)
        {
          current_kd[i] = kd_value;
        }
      }),
    mc_rtc::gui::Point3DRO("pt1", mc_rtc::gui::PointConfig(mc_rtc::gui::Color(0.0, 0.0, 1.0), 0.03), endEffectorTarget_pt1),
    mc_rtc::gui::Point3DRO("pt2", mc_rtc::gui::PointConfig(mc_rtc::gui::Color(0.0, 1.0, 0.0), 0.03), endEffectorTarget_pt2),
    mc_rtc::gui::Point3DRO("pt3", mc_rtc::gui::PointConfig(mc_rtc::gui::Color(1.0, 0.0, 0.0), 0.03), endEffectorTarget_pt3),
    mc_rtc::gui::Point3DRO("endEffectorTarget_pos", mc_rtc::gui::PointConfig(mc_rtc::gui::Color(1.0, 1.0, 0.0), 0.03), endEffectorTarget_pos)
    );

  mc_rtc::log::success("PostureDatastoreController init done ");
}

bool PostureDatastoreController::run()
{
  endEffectorTarget_pos = robot().mbc().bodyPosW[robot().bodyIndexByName(tool_frame)].translation();

  distance_pt1 = endEffectorTarget_pt1 - endEffectorTarget_pos;
  distance_pt2 = endEffectorTarget_pt2 - endEffectorTarget_pos;
  distance_pt3 = endEffectorTarget_pt3 - endEffectorTarget_pos;

  distance_pt1_norm = distance_pt1.norm();
  distance_pt2_norm = distance_pt2.norm();
  distance_pt3_norm = distance_pt3.norm();

  bool run = mc_control::fsm::Controller::run(mc_solver::FeedbackType::ClosedLoopIntegrateReal);
  robot().forwardKinematics();
  robot().forwardVelocity();
  robot().forwardAcceleration();
  
  if(isPureRL) // Run RL without taking account of the QP
  {
    // q_cmd = q_rl; **THIS IS NOT WORKING BECAUSE THE POLICY WAS NOT TRAINED WITH THE REAL PD OF THE ROBOT**
    // ** SOLUTION: Simulate the equivalent torque command**
    Eigen::MatrixXd Kp_inv = kp_robot.cwiseInverse().asDiagonal();
    Eigen::VectorXd Kd = kd_robot - kd_policy;
    tau_cmd = kp_policy.cwiseProduct(q_rl - currentPos) - kd_policy.cwiseProduct(currentVel);
    q_cmd = computeInversePD(tau_cmd);
    updateRobotCmdAfterQP();
    return true;
  }

  // Use QP
  computeQPAccelerationInversePD();
  updateRobotCmdAfterQP();
  return run; // Return false if QP fails
}

void PostureDatastoreController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}

void PostureDatastoreController::tasksComputation(void)
{
  auto & robot = robots()[0];
  auto & real_robot = realRobot(robots()[0].name());
  auto extTorqueSensor = robot.device<mc_rbdyn::VirtualTorqueSensor>("ExtTorquesVirtSensor");

  auto q = real_robot.encoderValues();
  currentPos = Eigen::VectorXd::Map(q.data(), q.size());
  auto vel = real_robot.encoderVelocities();
  currentVel = Eigen::VectorXd::Map(vel.data(), vel.size());

  tau_d = kp_policy.cwiseProduct(q_rl - currentPos) - kd_policy.cwiseProduct(currentVel);
  
  // Torque Task
  size_t i = 0;
  for (const auto &joint_name : jointNames)
  {
    torque_target[joint_name][0] = tau_d[i];
    i++;
  }

  // Forward Dynamics Task
  rbd::ForwardDynamics fd(real_robot.mb());
  fd.computeH(real_robot.mb(), real_robot.mbc());
  fd.computeC(real_robot.mb(), real_robot.mbc());
  Eigen::MatrixXd M = fd.H();
  Eigen::VectorXd Cg = fd.C();
  Eigen::VectorXd content = tau_d - Cg;
  if(!compensateExternalForces) content += extTorqueSensor.torques();
  refAccel = M.llt().solve(content);
}

void PostureDatastoreController::stiffnessAdjustment(void)
{
  double eval = postureTask->eval().norm();
  mc_rtc::log::info("[PostureDatastoreController] Posture Task eval: {}", eval);
  // Adjust stiffness based on the distance to the target position
  if(eval < 1.0)
  {
    double stiffness_posture_ = A * exp(k_slope * eval) + C; // Exponential
    mc_rtc::log::info("[PostureDatastoreController] Stiffness posture adjusted to: {}", stiffness_posture_);
    // damping_posture_ = 3*std::sqrt(stiffness_posture_);
    postureTask->stiffness(stiffness_posture_);
  }
}

void PostureDatastoreController::updateRobotCmdAfterQP()
{
  auto q = robot().mbc().q;
  auto alpha = robot().mbc().alpha;
  auto tau = robot().mbc().jointTorque;
  
  size_t i = 0;
  for (const auto &joint_name : jointNames)
  {
    q[robot().jointIndexByName(joint_name)][0] = q_cmd[i];
    alpha[robot().jointIndexByName(joint_name)][0] = 0.0;
    tau[robot().jointIndexByName(joint_name)][0] = tau_cmd[i];
    i++;
  }
  // Update q and qdot for position control
  robot().mbc().q = q;
  if(isRLQP | isPureRL) robot().mbc().alpha = alpha; // For RL policy qdot ref = 0
  // Update joint torques for torque control
  robot().mbc().jointTorque = tau;

  // Both are always updated despite they are not used by the robot
  // They are still used by the QP
}

void PostureDatastoreController::computeQPAccelerationInversePD()
{
  // Using QP (TorqueTask or ForwardDynamics Task):  
  rbd::paramToVector(robot().mbc().alphaD, ddot_qp);

  // Use robot instead of realrobot because we are after the QP
  rbd::ForwardDynamics fd(robot().mb());
  fd.computeH(robot().mb(), robot().mbc());
  fd.computeC(robot().mb(), robot().mbc());
  Eigen::MatrixXd M = fd.H();
  Eigen::VectorXd Cg = fd.C();

  auto extTorqueSensor = robot().device<mc_rbdyn::VirtualTorqueSensor>("ExtTorquesVirtSensor");
  tau_cmd = M*ddot_qp + Cg - extTorqueSensor.torques();
  
  Eigen::MatrixXd Kp_inv = current_kp.cwiseInverse().asDiagonal();

  q_cmd = computeInversePD(tau_cmd);
}

Eigen::VectorXd PostureDatastoreController::computeInversePD(Eigen::VectorXd tau)
{
  Eigen::MatrixXd Kp_inv = current_kp.cwiseInverse().asDiagonal();
  return currentPos + Kp_inv*(tau + current_kd.cwiseProduct(currentVel));
}

bool PostureDatastoreController::countPtReached()
{
  double distanceNorm_ptTarget = distance_pt2_norm;
  double distanceNorm_ptInit = distance_pt1_norm;

  if(distanceNorm_ptTarget < 0.000001 || distanceNorm_ptInit < 0.000001)
  {
    return false; // Check if value is different from zero
  }

  // Reset counter
  if(distanceNorm_ptInit > thresholdDistanceNorm && distanceNorm_ptTarget > thresholdDistanceNorm)
  {
    counterToValidateConvergence = 0.0;
  }

  if(distanceNorm_ptInit > thresholdDistanceNorm && convergedToPtInit)
  {
    convergedToPtInit = false;
  }

  if(distanceNorm_ptTarget > thresholdDistanceNorm && convergedToPtTarget)
  {
    convergedToPtTarget = false;
  }

  if(distanceNorm_ptTarget < thresholdDistanceNorm && !convergedToPtTarget)
  {
    counterToValidateConvergence += timeStep;
    if(counterToValidateConvergence >= thresholdToValidateConvergence)
    {
      convergedToPtTarget = true;
      mc_rtc::log::info("Reached ptTarget");
      if(!alreadyReachedPtTarget) alreadyReachedPtTarget = true;
    }
  }

  if(alreadyReachedPtTarget && distanceNorm_ptInit < thresholdDistanceNorm && !convergedToPtInit)
  {
    counterToValidateConvergence += timeStep;
    if(counterToValidateConvergence >= thresholdToValidateConvergence)
    {
      convergedToPtInit = true;
      counterPtReached++;
      mc_rtc::log::info("Reached ptInit");
      mc_rtc::log::info("Counter ptReached: {}", counterPtReached);
    }
  }

  if(counterPtReached >= 10)
  {
    mc_rtc::log::info(" Reached ptTarget 10 times, Changing state");
    return true; // Transition to the next state
  }
  return false;
}

void PostureDatastoreController::cleanState()
{
  compensateExternalForces = false;
  counterPtReached = 0;
  counterToValidateConvergence = 0.0;
  convergedToPtInit = false;
  convergedToPtTarget = false;
  alreadyReachedPtTarget = false;
}