#include "aav_control/gazebo_state_forwarder.h"

#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

namespace aav_control
{

GazeboStateForwarder::GazeboStateForwarder(QuinticControl &control, std::string modelName)
    : control_(control),
      model_name_(modelName)
{
}

void GazeboStateForwarder::forwardState(gazebo_msgs::ModelStatesConstPtr model_states)
{
  static int counter = 0;
  if (++counter < 99)
    return;
  counter = 0;

  size_t model_num;
  for (model_num = 0; model_num < model_states->name.size(); ++model_num)
    if (model_states->name[model_num] == "car")
      break;
  if (model_num == model_states->name.size())
    return;

  nav_msgs::OdometryPtr odometry(new nav_msgs::Odometry());
  odometry->pose.pose = model_states->pose[model_num];
  odometry->twist.twist = model_states->twist[model_num];
  control_.updateOdometry(nav_msgs::OdometryConstPtr(odometry));
}

}

