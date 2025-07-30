#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_tasks/CompliantPostureTask.h>
#include <mc_tasks/EndEffectorTask.h>

#include "api.h"
#include <Eigen/src/Core/Matrix.h>

struct PostureDatastoreController_DLLAPI PostureDatastoreController : public mc_control::fsm::Controller
{
  PostureDatastoreController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  std::map<std::string, std::vector<double>> posture;
  std::shared_ptr<mc_tasks::CompliantPostureTask> compPostureTask;
  std::shared_ptr<mc_tasks::EndEffectorTask> endEffectorTask_pt1;
  std::shared_ptr<mc_tasks::EndEffectorTask> endEffectorTask_pt2;
  std::shared_ptr<mc_tasks::EndEffectorTask> endEffectorTask_pt3;

  Eigen::Vector3d endEffectorTarget_pt1;
  Eigen::Vector3d endEffectorTarget_pt2;
  Eigen::Vector3d endEffectorTarget_pt3;

  void torqueTask(void);

  // std::shared_ptr<mc_tasks::PostureTask> FSMPostureTask;
  // std::shared_ptr<mc_tasks::PostureTask> similiTorqueTask;
  bool isRLQP = false; // Flag to indicate if the controller is in torque task + RL in position control
  bool isPureRL = false; // Flag to indicate if the controller is in RL mode in position control (without QP)

  std::vector<std::string> jointNames;

  Eigen::VectorXd refAccel;
  Eigen::VectorXd refPos;
  Eigen::VectorXd refPos_last; // Last reference position used in the RL controller
  Eigen::VectorXd tau_d;
  Eigen::VectorXd currentPos;
  Eigen::VectorXd currentVel;

  Eigen::VectorXd kp_vector;
  Eigen::VectorXd kd_vector;
  double kp_value;
  double kd_value;

  std::map<std::string, double> kp_position;
  std::map<std::string, double> kd_position;
  Eigen::VectorXd kp_position_vector;
  Eigen::VectorXd kd_position_vector;

  std::map<std::string, double> kp_torque;
  std::map<std::string, double> kd_torque;
  Eigen::VectorXd kp_torque_vector;
  Eigen::VectorXd kd_torque_vector;

  size_t dofNumber; // Number of degrees of freedom in the robot

  // For position control
  Eigen::VectorXd ddot_qp; // Desired acceleration in the QP solver
  Eigen::VectorXd q_cmd; // The commended position send to the internal PD of the robot
  Eigen::VectorXd tau_cmd_after_pd; // The commended position after PD control

  double stiffnessMin = 3.0;
  double stiffnessMax = 120.0;
  double k_slope = -3; // Slope for stiffness adjustment (Strictly negative for increasing stiffness)
  double A;
  double C;
  void stiffnessAdjustment(void); // stiffness = A* exp(k_slope * distance) + C

  double counter = 0.0; // Counter for the frequency of posture updates
  double t = 1.0; // end point at t=1 second.

private:
  mc_rtc::Configuration config_;
};
