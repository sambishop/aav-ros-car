#ifndef AAV_QUINTIC_PATH_CALCULATOR_H
#define	AAV_QUINTIC_PATH_CALCULATOR_H

#include <aav_msgs/QuinticPath.h>
#include <aav_quintic/path_segment_calculator.h>
#include <tf2/LinearMath/Vector3.h>
#include <vector>

namespace aav_quintic
{

class PathCalculator
{
public:
  PathCalculator(const aav_msgs::QuinticPath &path);
  ~PathCalculator();
  tf2::Vector3 calculate(unsigned int segment_index, double t) const;
  tf2::Vector3 calculate1stDerivative(unsigned int segment_index, double t) const;
  unsigned int numSegments() const;

private:
  std::vector<const aav_quintic::PathSegmentCalculator *> calculators_;
};

} // end namespace aav_quintic

#endif

