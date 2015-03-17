#include <stdio.h>

#include "QuinticControl.h"

using namespace aav_control;
using namespace nav_msgs;

void QuinticControl::updateGoal(const DoQuinticPathGoalConstPtr &goal)
{ 
  this->goal = goal;
}

void QuinticControl::updateOdometry(const Odometry::ConstPtr &odometry)
{
  if (!goal) {
    return;
  }

  fprintf(stderr, "%s\n", odometry->child_frame_id.c_str());
}

