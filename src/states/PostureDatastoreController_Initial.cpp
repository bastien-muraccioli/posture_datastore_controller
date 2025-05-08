#include "PostureDatastoreController_Initial.h"

#include "../PostureDatastoreController.h"

void PostureDatastoreController_Initial::configure(const mc_rtc::Configuration & config) {}

void PostureDatastoreController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PostureDatastoreController &>(ctl_);
}

bool PostureDatastoreController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PostureDatastoreController &>(ctl_);
  output("OK");
  return true;
}

void PostureDatastoreController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PostureDatastoreController &>(ctl_);
}

EXPORT_SINGLE_STATE("PostureDatastoreController_Initial", PostureDatastoreController_Initial)
