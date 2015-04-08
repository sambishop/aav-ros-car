#include "PathCalculator.h"

namespace aav_control
{

PathCalculator::PathCalculator(const aav_msgs::QuinticPath& path) : path_(path)
{
  for (unsigned int i = 0; i < path.segments.size(); ++i)
    calculators_.push_back(new PathSegmentCalculator(path.segments[i]));
}

PathCalculator::~PathCalculator()
{
  for (unsigned int i = 0; i < calculators_.size(); ++i)
    delete calculators_[i];
}

tf2::Vector3 PathCalculator::calculate(const PathPoint& point) const
{
  return calculators_[point.getSegment()]->calculate(point.getT());
}

} // end namespace aav_control

