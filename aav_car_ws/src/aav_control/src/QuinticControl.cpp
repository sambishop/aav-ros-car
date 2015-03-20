#include <gsl/gsl_errno.h>
#include <stdio.h>
#include <vector>

#include "aav_msgs/QuinticPath.h"
#include "DistanceCalculator.h"
#include "geometry_msgs/Point.h"
#include "QuinticControl.h"

using namespace aav_control;
using namespace aav_msgs;
using namespace geometry_msgs;
using namespace nav_msgs;
using namespace std;

QuinticControl::QuinticControl() {
  gsl_set_error_handler_off();
}

void QuinticControl::updateGoal(const DoQuinticPathGoalConstPtr &goal) {
  goalMutex.lock();
  this->goal = goal;
  goalMutex.unlock();
}

const QuinticPath *QuinticControl::getPathFromGoal() {
  const QuinticPath *path = NULL;
  goalMutex.lock();
  if (goal) {
    path = &goal->path;
  }
  goalMutex.unlock();
  return path;
}

void QuinticControl::updateOdometry(const Odometry::ConstPtr &odometry) {
  const QuinticPath *path = getPathFromGoal();
  if (path == NULL || path->segments.empty()) {
    return;
  }

  const Point *position = &odometry->pose.pose.position;
  DistanceCalculator calculator(&path->segments[0], position);
  double t = calculator.calculate();
  fprintf(stderr, "t = %f\n", t);
}

