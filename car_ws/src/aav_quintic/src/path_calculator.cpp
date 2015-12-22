#include "aav_quintic/path_calculator.h"

namespace aav_quintic
{

PathCalculator::PathCalculator(const aav_msgs::QuinticPath &path)
{
  for (unsigned int i = 0; i < path.segments.size(); ++i)
    calculators_.push_back(new aav_quintic::PathSegmentCalculator(path.segments[i]));
}

PathCalculator::~PathCalculator()
{
  for (unsigned int i = 0; i < calculators_.size(); ++i)
    delete calculators_[i];
}

tf2::Vector3 PathCalculator::calculate(unsigned int segment_index, double t) const
{
  return calculators_[segment_index]->calculate(t);
}

tf2::Vector3 PathCalculator::calculate1stDerivative(unsigned int segment_index, double t) const
{
  return calculators_[segment_index]->calculate1stDerivative(t);
}

unsigned int PathCalculator::numSegments() const
{
  return calculators_.size();
}

} // end namespace aav_quintic

