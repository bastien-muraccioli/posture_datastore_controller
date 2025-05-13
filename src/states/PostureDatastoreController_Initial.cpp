#include "PostureDatastoreController_Initial.h"

#include "../PostureDatastoreController.h"

void PostureDatastoreController_Initial::configure(const mc_rtc::Configuration & config) {}

void PostureDatastoreController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PostureDatastoreController &>(ctl_);
  // ctl.compPostureTask->damping(100);
}

bool PostureDatastoreController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PostureDatastoreController &>(ctl_);
  
  if (ctl.datastore().has("ros_posture_pub_sub"))
  {
    auto posture = ctl.datastore().get<std::map<std::string, std::vector<double>>>("ros_posture_pub_sub");
    if (posture.size() > 0)
    {
      ctl.compPostureTask->target(posture);
    }
  }  
  // output("OK");
  return false;
}

void PostureDatastoreController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PostureDatastoreController &>(ctl_);
}

EXPORT_SINGLE_STATE("PostureDatastoreController_Initial", PostureDatastoreController_Initial)
