#ifndef AAV_CONTROL_PATH_SEGMENT_CALCULATOR_H
#define	AAV_CONTROL_PATH_SEGMENT_CALCULATOR_H

#include <aav_msgs/QuinticPathSegment.h>
#include <tf2/LinearMath/Vector3.h>

#include "spline_segment_calculator.h"

namespace aav_control
{

class PathSegmentCalculator
{
public:
  PathSegmentCalculator(const aav_msgs::QuinticPathSegment &segment);
  tf2::Vector3 calculate(double t) const;

private:
  const aav_msgs::QuinticPathSegment &segment_;
  const SplineSegmentCalculator x_calculator_;
  const SplineSegmentCalculator y_calculator_;
};

} // end namespace aav_control

#endif

