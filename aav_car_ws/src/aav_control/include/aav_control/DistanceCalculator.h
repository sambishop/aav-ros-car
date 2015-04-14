#ifndef _DISTANCE_CALCULATOR_H_
#define _DISTANCE_CALCULATOR_H_

#include <gsl/gsl_min.h>

#include "aav_msgs/QuinticPathSegment.h"
#include "aav_msgs/QuinticSplineSegment.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

class DistanceCalculator {
public:
  DistanceCalculator(
      const aav_msgs::QuinticPathSegment *segment,
      const geometry_msgs::Point *point
    );
  ~DistanceCalculator();
  double findT();
  double calculateCrossTrackError(double t, const geometry_msgs::Pose &pose);

private:
  static double calculateDistanceMeasure(double t, void *);
  double calculateSegment(
      double t,
      const aav_msgs::QuinticSplineSegment *segment
    );
  tf2::Quaternion extractOrientation(const geometry_msgs::Pose &pose);
  tf2::Vector3 extractPosition(const geometry_msgs::Pose &pose);
  tf2::Vector3 calculatePosition(double t);
  const aav_msgs::QuinticPathSegment *segment;
  const geometry_msgs::Point *point;
  gsl_min_fminimizer *minimizer;
  gsl_function func;
};

#endif

