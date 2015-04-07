#ifndef AAV_CONTROL_PATHSEGMENTCALCULATOR_H
#define	AAV_CONTROL_PATHSEGMENTCALCULATOR_H

#include <aav_msgs/QuinticPathSegment.h>
#include <tf2/LinearMath/Vector3.h>

#include "SplineSegmentCalculator.h"

namespace aav_control
{

class PathSegmentCalculator
{
public:
  PathSegmentCalculator(const aav_msgs::QuinticPathSegment &segment);
  tf2::Vector3 calculate(double t) const;

private:
  const aav_msgs::QuinticPathSegment &segment_;
  const aav_control::SplineSegmentCalculator x_calculator_;
  const aav_control::SplineSegmentCalculator y_calculator_;
};

} // end namespace aav_control

#endif

