#include <stdio.h>

#include "aav_msgs/QuinticPath.h"
#include "geometry_msgs/Point.h"
#include "QuinticControl.h"

using namespace aav_control;
using namespace aav_msgs;
using namespace geometry_msgs;
using namespace nav_msgs;

void QuinticControl::updateGoal(const DoQuinticPathGoalConstPtr &goal)
{ 
  goalMutex.lock();
  this->goal = goal;
  goalMutex.unlock();
}

void QuinticControl::updateOdometry(const Odometry::ConstPtr &odometry)
{
  goalMutex.lock();
  if (!goal) {
    goalMutex.unlock();
    return;
  }
  const QuinticPath *path = &goal->path;
  goalMutex.unlock();

  const Point *position = &odometry->pose.pose.position;
}

