#include "PostureDatastoreController.h"
#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/NumberInput.h>
#include <mc_rtc/gui/Rotation.h>
#include <string>

PostureDatastoreController::PostureDatastoreController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{

  tool_frame = config("tool_frame", (std::string) "end_effector_link");

  // Initialize the constraints
  selfCollisionConstraint->setCollisionsDampers(solver(), {1.8, 70.0});
  solver().removeConstraintSet(dynamicsConstraint);
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
    new mc_solver::DynamicsConstraint(robots(), 0, {0.1, 0.01, 0.0, 1.8, 70.0}, 0.9, true));
  solver().addConstraintSet(dynamicsConstraint);

  // Default posture target
  // posture = {{"joint_1", {0}}, {"joint_2", {0.262}}, {"joint_3", {3.14}}, {"joint_4", {-2.269}},
  //                  {"joint_5", {0}}, {"joint_6", {0.96}},  {"joint_7", {1.57}}};

  // posture during the training
  posture = {{"joint_1", {0.0}}, {"joint_2", {0.65}}, {"joint_3", {0.0}}, {"joint_4", {1.89}},
                   {"joint_5", {0.0}}, {"joint_6", {0.6}},  {"joint_7", {-1.57}}};

  torque = {{"joint_1", {0.0}}, {"joint_2", {0.0}}, {"joint_3", {0.0}}, {"joint_4", {0.0}},
                   {"joint_5", {0.0}}, {"joint_6", {0.0}},  {"joint_7", {0.0}}};

  endEffectorTarget_pt1 = Eigen::Vector3d(0.5, 0.0, 0.2);
  endEffectorTarget_pt2 = Eigen::Vector3d(0.4, -0.15, 0.3);
  endEffectorTarget_pt3 = Eigen::Vector3d(0.6, 0.1, 0.45);

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
  refPos = Eigen::VectorXd::Zero(dofNumber);
  refPos_last = Eigen::VectorXd::Zero(dofNumber); // Initialize last reference position
  tau_d = Eigen::VectorXd::Zero(dofNumber);
  currentPos = Eigen::VectorXd::Zero(dofNumber);
  currentVel = Eigen::VectorXd::Zero(dofNumber);
  kp_vector = Eigen::VectorXd::Zero(dofNumber);
  kd_vector = Eigen::VectorXd::Zero(dofNumber);
  kp_position_vector = Eigen::VectorXd::Zero(dofNumber);
  kd_position_vector = Eigen::VectorXd::Zero(dofNumber);
  kp_torque_vector = Eigen::VectorXd::Zero(dofNumber);
  kd_torque_vector = Eigen::VectorXd::Zero(dofNumber);
  ddot_qp = Eigen::VectorXd::Zero(dofNumber); // Desired acceleration in the QP solver
  q_cmd = Eigen::VectorXd::Zero(dofNumber); // The commended position send to the internal PD of the robot
  tau_cmd_after_pd = Eigen::VectorXd::Zero(dofNumber); // The commended position after PD control

  // Compute A and C for stiffness adjustment
  A = (-stiffnessMax + stiffnessMin) / (exp(k_slope) - 1);
  C = stiffnessMax - A;
  
  kp_position = config("kp_position");
  kd_position = config("kd_position");
  kp_torque = config("kp_torque");
  kd_torque = config("kd_torque");

  size_t i = 0;
  for (const auto &j : robot().mb().joints()) {
      const std::string &joint_name = j.name();
      if(j.type() == rbd::Joint::Type::Rev)
      {
        jointNames.emplace_back(joint_name);
        if (const auto &t = posture[joint_name]; !t.empty()) {
            kp_vector[i] = kp_position.at(joint_name);
            kd_vector[i] = kd_position.at(joint_name);
            kp_position_vector[i] = kp_position.at(joint_name);
            kd_position_vector[i] = kd_position.at(joint_name);
            kp_torque_vector[i] = kp_torque.at(joint_name);
            kd_torque_vector[i] = kd_torque.at(joint_name);
            refPos[i] = t[0];
            mc_rtc::log::info("[PostureDatastoreController] Joint {}: refPos {}, kp {}, kd {}", joint_name, refPos[i], kp_vector[i], kd_vector[i]);
            i++;
        }
      }
  }

  kp_value = kp_vector[0]; // Assuming all joints have the same kp value
  kd_value = kd_vector[0]; // Assuming all joints have the same kd value
  
  // Remove the default posture task created by the FSM
  solver().removeTask(getPostureTask(robot().name()));

  compPostureTask = std::make_shared<mc_tasks::PostureTask>(solver(), robot().robotIndex(), 1, 1);
  compPostureTask->target(posture);
  // compPostureTask->stiffness(stiffnessMin);
  // solver().addTask(compPostureTask);

  torqueTask = std::make_shared<mc_tasks::TorqueTask>(solver(), robot().robotIndex());

  logger().addLogEntry("RLController_refAccel", [this]() { return refAccel; });
  // logger().addLogEntry("RLController_refAccel_w_floatingBase", [this]()
  // { return refAccel_w_floatingBase; });
  logger().addLogEntry("RLController_refPos", [this]() { return refPos; });
  logger().addLogEntry("RLController_tau_d", [this]() { return tau_d; });
  // logger().addLogEntry("RLController_tau_d_w_floatingBase", [this]()
  // { return tau_d_w_floatingBase; });
  logger().addLogEntry("RLController_kp", [this]() { return kp_vector; });
  logger().addLogEntry("RLController_kd", [this]() { return kd_vector; });
  logger().addLogEntry("RLController_currentPos", [this]() { return currentPos; });
  // logger().addLogEntry("RLController_currentPos_w_floatingBase", [this]()
  // { return currentPos_w_floatingBase; });
  logger().addLogEntry("RLController_currentVel", [this]() { return currentVel; });
  logger().addLogEntry("RLController_q_cmd", [this]() { return q_cmd; });
  // logger().addLogEntry("RLController_q_cmd_w_floatingBase", [this]()
  // { return q_cmd_w_floatingBase; });
  logger().addLogEntry("RLController_ddot_qp", [this]() { return ddot_qp; });

  logger().addLogEntry("RLController_tau_cmd_after_pd_positionCtl", [this]() { return tau_cmd_after_pd; });
  logger().addLogEntry("RLController_distance_pt1", [this]() { return distance_pt1; });
  logger().addLogEntry("RLController_distance_pt2", [this]() { return distance_pt2; });
  logger().addLogEntry("RLController_distance_pt3", [this]() { return distance_pt3; });
  logger().addLogEntry("RLController_distance_pt1_norm", [this]() { return distance_pt1_norm; });
  logger().addLogEntry("RLController_distance_pt2_norm", [this]() { return distance_pt2_norm; });
  logger().addLogEntry("RLController_distance_pt3_norm", [this]() { return distance_pt3_norm; });
  logger().addLogEntry("RLController_endEffectorTarget_pos", [this]() { return endEffectorTarget_pos; });

  // Kinova Gen3 datastore
  datastore().make<std::string>("ControlMode", "Torque");
  datastore().make<std::string>("TorqueMode", "Custom");
  datastore().make_call("getPostureTask", [this]() -> mc_tasks::PostureTaskPtr { return compPostureTask; });

  // Add GUI to modify kp and kd
  gui()->addElement({"PostureDatastoreController"},
    mc_rtc::gui::ArrayInput("Kp Vector",
      [this]() { return kp_vector; },
      [this](const Eigen::VectorXd & v) { kp_vector = v; }),
    mc_rtc::gui::ArrayInput("Kd Vector",
      [this]() { return kd_vector; },
      [this](const Eigen::VectorXd & v) { kd_vector = v; }),
    mc_rtc::gui::NumberInput("Kp",
      [this]() { return kp_value; },
      [this](const double v) {
        kp_value = v;
        for(size_t i = 0; i < kp_vector.size(); ++i)
        {
          kp_vector[i] = kp_value;
        }
      }),
    mc_rtc::gui::NumberInput("Kd",
      [this]() { return kd_value; },
      [this](const double v) {
        kd_value = v;
        for(size_t i = 0; i < kd_vector.size(); ++i)
        {
          kd_vector[i] = kd_value;
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
  if(compensateExternalForcesHasChanged != compensateExternalForces)
  {
    mc_rtc::log::info("[PostureDatastoreController] Compensate external forces: {}", compensateExternalForces);
    compensateExternalForcesHasChanged = compensateExternalForces; // Update the flag to the current state
  }

  endEffectorTarget_pos = robot().mbc().bodyPosW[robot().bodyIndexByName(tool_frame)].translation();

  distance_pt1 = endEffectorTarget_pt1 - endEffectorTarget_pos;
  distance_pt2 = endEffectorTarget_pt2 - endEffectorTarget_pos;
  distance_pt3 = endEffectorTarget_pt3 - endEffectorTarget_pos;

  distance_pt1_norm = distance_pt1.norm();
  distance_pt2_norm = distance_pt2.norm();
  distance_pt3_norm = distance_pt3.norm();
  
  auto ctrl_mode = datastore().get<std::string>("ControlMode");
  if(ctrl_mode.compare("Position") == 0)
  {
    // Position control mode with RL and QP
    if(isRLQP)
    {
      auto run = mc_control::fsm::Controller::run(mc_solver::FeedbackType::ClosedLoopIntegrateReal);
      robot().forwardKinematics();
      robot().forwardVelocity();
      robot().forwardAcceleration();
      rbd::paramToVector(robot().mbc().alphaD, ddot_qp);

      // Use robot instead of realrobot because we are after the QP
      rbd::ForwardDynamics fd(robot().mb());
      fd.computeH(robot().mb(), robot().mbc());
      fd.computeC(robot().mb(), robot().mbc());
      Eigen::MatrixXd M = fd.H();
      Eigen::VectorXd Cg = fd.C();

      Eigen::MatrixXd Kp_inv = kp_vector.cwiseInverse().asDiagonal();

      // auto extTorqueSensor = robot().device<mc_rbdyn::VirtualTorqueSensor>("ExtTorquesVirtSensor");
      // Eigen::VectorXd externalTorques = Eigen::VectorXd::Zero(dofNumber);
      // Eigen::VectorXd torques = extTorqueSensor.torques();
      // if (torques.hasNaN()) {
      //   mc_rtc::log::error("[RLController] External torques contain NaN values, using zero torques instead");
      // } else {
      //   externalTorques = torques;
      // }
      Eigen::VectorXd externalTorques = Eigen::VectorXd::Zero(dofNumber);
      if(!compensateExternalForces)
      {
        auto extTorqueSensor = robot().device<mc_rbdyn::VirtualTorqueSensor>("ExtTorquesVirtSensor");
         externalTorques = extTorqueSensor.torques();
      }

      // q_cmd = currentPos + Kp_inv*(M*ddot_qp + Cg - externalTorques + kd_vector.cwiseProduct(currentVel)); // Inverse PD control to get the commanded position <=> RL position control
      q_cmd = currentPos + Kp_inv*(M*ddot_qp + Cg - externalTorques + kd_vector.cwiseProduct(currentVel));
      tau_cmd_after_pd = kd_vector.cwiseProduct(currentPos - q_cmd) - kd_vector.cwiseProduct(currentVel); // PD control to get the commanded position after PD control
      mc_rtc::log::info("[PostureDatastoreController] q_cmd {}, tau_cmd_after_pd {}",q_cmd, tau_cmd_after_pd);
    
      auto q = robot().mbc().q;
      
      size_t i = 0;
      for (const auto &joint_name : jointNames)
      {
        q[robot().jointIndexByName(joint_name)][0] = q_cmd[i];
        i++;
      }

      robot().mbc().q = q; // Update the mbc with the new position
      return run;
    }
  // Position control mode (without TorqueTask or RL)
  return mc_control::fsm::Controller::run(mc_solver::FeedbackType::OpenLoop);
  }

  else
  {
    // Torque control mode with the RL command (Without QP)
    if(isPureRL)
    {

      mc_control::fsm::Controller::run(mc_solver::FeedbackType::ClosedLoopIntegrateReal);
      robot().forwardKinematics();
      robot().forwardVelocity();
      robot().forwardAcceleration();
      auto q = robot().encoderValues();
      currentPos = Eigen::VectorXd::Map(q.data(), q.size());
      auto vel = robot().encoderVelocities();
      currentVel = Eigen::VectorXd::Map(vel.data(), vel.size());
      auto tau = robot().mbc().jointTorque;
      tau_d = kp_vector.cwiseProduct(refPos - currentPos) + kd_vector.cwiseProduct(-currentVel);
      // counter += timeStep;
      // if(refPos_last != refPos)
      // {
      //   refPos_last = refPos; // Update the last reference position
      
        size_t i = 0;
        for (const auto &joint_name : jointNames)
        {
          tau[robot().jointIndexByName(joint_name)][0] = tau_d[i];
          i++;
        }
        
      // }

      robot().mbc().jointTorque = tau; // Update the mbc with the new position
      return true;
      // }
      // return false;
    }
  }
  // Torque control mode
  return mc_control::fsm::Controller::run(mc_solver::FeedbackType::ClosedLoopIntegrateReal);
}

void PostureDatastoreController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}

void PostureDatastoreController::FDTask(void)
{
  auto & robot = robots()[0];
  auto & real_robot = realRobot(robots()[0].name());

  auto q = real_robot.encoderValues();
  currentPos = Eigen::VectorXd::Map(q.data(), q.size());
  auto vel = real_robot.encoderVelocities();
  currentVel = Eigen::VectorXd::Map(vel.data(), vel.size());

  // mc_rtc::log::info("[RLController] Current Position: {}", currentPos);
  // mc_rtc::log::info("[RLController] Current Velocity: {}", currentVel);

  tau_d = kp_vector.cwiseProduct(refPos - currentPos) + kd_vector.cwiseProduct(-currentVel);
  // Update the torque target in the torque task
  size_t i = 0;
  for (const auto &joint_name : jointNames)
  {
    torque[joint_name][0] = tau_d[i];
    i++;
  }

  if(!compensateExternalForces)
  {
    auto extTorqueSensor = robot.device<mc_rbdyn::VirtualTorqueSensor>("ExtTorquesVirtSensor");
    Eigen::VectorXd externalTorques = extTorqueSensor.torques();

    tau_d += externalTorques;
  }
  // mc_rtc::log::info("[RLController] tau_d: {}", tau_d);

  // Simulate the torque task by converting the torque target to an acceleration target
  rbd::ForwardDynamics fd(real_robot.mb());
  fd.computeH(real_robot.mb(), real_robot.mbc());
  fd.computeC(real_robot.mb(), real_robot.mbc());
  Eigen::MatrixXd M = fd.H();
  Eigen::VectorXd Cg = fd.C();
  // Eigen::MatrixXd M_w_floatingBase = fd.H();
  // Eigen::VectorXd Cg_w_floatingBase = fd.C();
  // Eigen::MatrixXd M = M_w_floatingBase.bottomRightCorner(dofNumber, dofNumber);
  // Eigen::VectorXd Cg = Cg_w_floatingBase.tail(dofNumber);
  refAccel = M.completeOrthogonalDecomposition().solve(tau_d - Cg);

  // mc_rtc::log::info("[PostureDatastoreController] refAccel: {}", refAccel);
}

void PostureDatastoreController::stiffnessAdjustment(void)
{
  double eval = compPostureTask->eval().norm();
  mc_rtc::log::info("[PostureDatastoreController] Posture Task eval: {}", eval);
  // Adjust stiffness based on the distance to the target position
  if(eval < 1.0)
  {
    double stiffness_posture_ = A * exp(k_slope * eval) + C; // Exponential
    mc_rtc::log::info("[PostureDatastoreController] Stiffness posture adjusted to: {}", stiffness_posture_);
    // damping_posture_ = 3*std::sqrt(stiffness_posture_);
    compPostureTask->stiffness(stiffness_posture_);
  }
}