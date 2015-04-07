#ifndef AAV_CONTROL_PATHCALCULATOR_H
#define	AAV_CONTROL_PATHCALCULATOR_H

#include <aav_msgs/QuinticPath.h>
#include <tf2/LinearMath/Vector3.h>

#include "PathPoint.h"
#include "PathSegmentCalculator.h"

namespace aav_control
{

class PathCalculator
{
public:
  PathCalculator(const aav_msgs::QuinticPath &path);
  ~PathCalculator();
  tf2::Vector3 calculate(const aav_control::PathPoint &point) const;

private:
  const aav_msgs::QuinticPath &path_;
  std::vector<const aav_control::PathSegmentCalculator *> calculators_;
};

} // end namespace aav_control

#endif

