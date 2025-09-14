#pragma once

#include <mc_control/fsm/State.h>

struct PostureDatastoreController_DefaultPosture : mc_control::fsm::State
{

  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  bool isRobotStopped = false; // Flag to indicate if the robot is stopped
  bool isReachedTarget = false; // Flag to indicate if the target position is reached

private:
};
