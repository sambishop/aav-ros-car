#include <gsl/gsl_errno.h>
#include <stdio.h>
#include <vector>

#include "aav_msgs/QuinticPath.h"
#include "ackermann_msgs/AckermannDrive.h"
#include "DistanceCalculator.h"
#include "geometry_msgs/Point.h"
#include "QuinticControl.h"

using namespace aav_control;
using namespace aav_msgs;
using namespace geometry_msgs;
using namespace nav_msgs;
using namespace ros;
using namespace std;

QuinticControl::QuinticControl(Publisher *pub) : pid(0, .1, .1, .1, -4, 4)
{
  this->pub = pub;
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
  unsigned int i;
  double t;
  double crossTrackError;
  for (i = 0; i < path->segments.size(); ++i) {
    DistanceCalculator calculator(&path->segments[i], position);
    t = calculator.findT();
    if (t <= 1.0) {
      crossTrackError = calculator.calculateCrossTrackError(t);
      break;
    }
  }
  if (i == path->segments.size()) {
    return;
  }
  fprintf(stderr, "t=%f, crossTrackError=%f\n", t, crossTrackError);
}

