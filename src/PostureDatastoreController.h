#pragma once

#include <mc_control/fsm/Controller.h>

#include "api.h"

struct PostureDatastoreController_DLLAPI PostureDatastoreController : public mc_control::fsm::Controller
{
  PostureDatastoreController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

private:
  mc_rtc::Configuration config_;
};
