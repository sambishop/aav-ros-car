#ifndef _QUINTIC_CONTROL_H_
#define _QUINTIC_CONTROL_H_

#include "nav_msgs/Odometry.h"
#include "ros/ros.h"

namespace aav_control {
  class QuinticControl {
  public:
    void updateOdometry(const nav_msgs::Odometry::ConstPtr &odometry);
  };
}

#endif

