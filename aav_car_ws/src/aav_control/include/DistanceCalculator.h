#ifndef _DISTANCE_CALCULATOR_H_
#define _DISTANCE_CALCULATOR_H_

#include <gsl/gsl_min.h>

#include "aav_msgs/QuinticPathSegment.h"
#include "aav_msgs/QuinticSplineSegment.h"
#include "geometry_msgs/Point.h"

class DistanceCalculator {
public:
  DistanceCalculator(
      const aav_msgs::QuinticPathSegment *segment,
      const geometry_msgs::Point *point
    );
  ~DistanceCalculator();
  double findT();
  double calculateCrossTrackError(double t);

private:
  static double calculateDistanceMeasure(double t, void *);
  double calculateSegment(
      double t,
      const aav_msgs::QuinticSplineSegment *segment
    );
  const aav_msgs::QuinticPathSegment *segment;
  const geometry_msgs::Point *point;
  gsl_min_fminimizer *minimizer;
  gsl_function func;
};

#endif

