#ifndef AAV_QUINTIC_PATH_SEGMENT_CALCULATOR_H
#define	AAV_QUINTIC_PATH_SEGMENT_CALCULATOR_H

#include <aav_msgs/QuinticPathSegment.h>
#include <tf2/LinearMath/Vector3.h>

#include "aav_quintic/spline_segment_calculator.h"

namespace aav_quintic
{

class PathSegmentCalculator
{
public:
  PathSegmentCalculator(const aav_msgs::QuinticPathSegment &segment);
  tf2::Vector3 calculate(double t) const;
  tf2::Vector3 calculate1stDerivative(double t) const;

private:
  const aav_msgs::QuinticPathSegment &segment_;
  const SplineSegmentCalculator x_calculator_;
  const SplineSegmentCalculator y_calculator_;
};

} // end namespace aav_quintic

#endif

