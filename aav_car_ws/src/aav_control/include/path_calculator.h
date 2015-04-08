#ifndef AAV_CONTROL_PATH_CALCULATOR_H
#define	AAV_CONTROL_PATH_CALCULATOR_H

#include <aav_msgs/QuinticPath.h>
#include <tf2/LinearMath/Vector3.h>

#include "path_point.h"
#include "path_segment_calculator.h"

namespace aav_control
{

class PathCalculator
{
public:
  PathCalculator(const aav_msgs::QuinticPath &path);
  ~PathCalculator();
  tf2::Vector3 calculate(const PathPoint &point) const;

private:
  const aav_msgs::QuinticPath &path_;
  std::vector<const PathSegmentCalculator *> calculators_;
};

} // end namespace aav_control

#endif

