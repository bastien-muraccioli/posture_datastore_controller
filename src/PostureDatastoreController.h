#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_tasks/CompliantPostureTask.h>
#include <mc_tasks/PostureTask.h>
#include <mc_tasks/TorqueTask.h>

#include "api.h"
#include <Eigen/src/Core/Matrix.h>

struct PostureDatastoreController_DLLAPI PostureDatastoreController : public mc_control::fsm::Controller
{
  PostureDatastoreController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  bool countPtReached();

  void cleanState();

  std::string tool_frame;

  std::map<std::string, std::vector<double>> posture_init_rl;
  std::map<std::string, std::vector<double>> posture_init_default;
  std::map<std::string, std::vector<double>> torque_target;
  std::shared_ptr<mc_tasks::PostureTask> postureTask;
  std::shared_ptr<mc_tasks::TorqueTask> torqueTask;

  Eigen::Vector3d endEffectorTarget_pt1;
  Eigen::Vector3d endEffectorTarget_pt2;
  Eigen::Vector3d endEffectorTarget_pt3;

  // When the policy 
  Eigen::Vector3d endEffectorTarget_pt1_OOD;
  Eigen::Vector3d endEffectorTarget_pt2_OOD;
  Eigen::Vector3d endEffectorTarget_pt3_OOD;
  
  
  Eigen::Vector3d distance_pt1;
  Eigen::Vector3d distance_pt2;
  Eigen::Vector3d distance_pt3;

  Eigen::Vector3d endEffectorTarget_ptTarget;
  Eigen::Vector3d endEffectorTarget_ptInit;
  Eigen::Vector3d distance_ptTarget;
  Eigen::Vector3d distance_ptInit;

  double counterToValidateConvergence = 0.0;
  double thresholdToValidateConvergence = 3.0; // 3s
  double thresholdDistanceNorm = 0.05; // 5cm
  int counterPtReached = 0;

  bool convergedToPtTarget = false;
  bool alreadyReachedPtTarget = false;
  bool convergedToPtInit = false;

  Eigen::Vector3d endEffectorTarget_pos;

  double distance_pt1_norm = 0.0;
  double distance_pt2_norm = 0.0;
  double distance_pt3_norm = 0.0;

  void tasksComputation(void);

  // std::shared_ptr<mc_tasks::PostureTask> FSMPostureTask;
  // std::shared_ptr<mc_tasks::PostureTask> similiTorqueTask;
  bool isRLQP = false; // Flag to indicate if the controller is in torque task + RL in position control
  bool isPureRL = false; // Flag to indicate if the controller is in RL mode in position control (without QP)

  std::vector<std::string> jointNames;

  Eigen::VectorXd refAccel;
  Eigen::VectorXd q_rl;
  Eigen::VectorXd q_rl_last; // Last reference position used in the RL controller
  Eigen::VectorXd tau_d;
  Eigen::VectorXd currentPos;
  Eigen::VectorXd currentVel;

  Eigen::VectorXd current_kp;
  Eigen::VectorXd current_kd;
  double kp_value;
  double kd_value;

  Eigen::VectorXd kp_robot;
  Eigen::VectorXd kd_robot;
  Eigen::VectorXd kp_policy;
  Eigen::VectorXd kd_policy;

  size_t dofNumber; // Number of degrees of freedom in the robot

  // For position control
  Eigen::VectorXd ddot_qp; // Desired acceleration in the QP solver
  Eigen::VectorXd q_cmd; // The commended position send to the internal PD of the robot
  Eigen::VectorXd tau_cmd; // The commended position after PD control

  double stiffnessMin = 3.0;
  double stiffnessMax = 120.0;
  double k_slope = -3; // Slope for stiffness adjustment (Strictly negative for increasing stiffness)
  double A;
  double C;
  void stiffnessAdjustment(void); // stiffness = A* exp(k_slope * distance) + C

  double counter = 0.0; // Counter for the frequency of posture updates
  double t = 1.0; // end point at t=1 second.

  bool compensateExternalForces = false; // Flag to indicate if the torque task should compensate external forces
  bool compensateExternalForcesHasChanged = false; // Flag to indicate if the external force compensation has changed

  void updateRobotCmdAfterQP();
  Eigen::VectorXd computeInversePD(Eigen::VectorXd tau);
  void computeQPAccelerationInversePD(); // Update q_cmd based on QP acceleration
  std::tuple<Eigen::VectorXd, Eigen::VectorXd> getPDGains();
  bool setPDGains(Eigen::VectorXd p_vec, Eigen::VectorXd d_vec);
  bool isHighGain(double tol = 1e-9);

private:
  mc_rtc::Configuration config_;
};
