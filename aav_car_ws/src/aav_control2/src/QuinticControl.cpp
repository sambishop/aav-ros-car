#include "QuinticControl.h"

using namespace aav_control;

QuinticControl::QuinticControl()
{
}

void QuinticControl::updateOdometry(const nav_msgs::Odometry::ConstPtr &odometry) {
  fprintf(stderr, "updateOdometry() called\n");
}

