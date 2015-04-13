#include "path_segment_calculator.h"

namespace aav_control
{

PathSegmentCalculator::PathSegmentCalculator(const aav_msgs::QuinticPathSegment &segment)
  : segment_(segment), x_calculator_(segment.x_segment), y_calculator_(segment.y_segment)
{}

tf2::Vector3 PathSegmentCalculator::calculate(double t) const
{
  tf2::Vector3 position(x_calculator_.calculate(t), y_calculator_.calculate(t), 0);
  return position;
}

tf2::Vector3 PathSegmentCalculator::calculate1stDerivative(double t) const
{
  tf2::Vector3 position(
      x_calculator_.calculate1stDerivative(t),
      y_calculator_.calculate1stDerivative(t),
      0
    );
  return position;
}

} // end namespace aav_control

