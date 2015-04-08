#ifndef AAV_CONTROL_CTE_CALCULATOR_H
#define	AAV_CONTROL_CTE_CALCULATOR_H

#include <aav_msgs/QuinticPath.h>
#include <tf2/LinearMath/Vector3.h>
#include <utility>

#include "path_calculator.h"
#include "path_point.h"

namespace aav_control
{

class CteCalculator
{
public:
  CteCalculator(const aav_msgs::QuinticPath &path);
  double calculate(const tf2::Vector3 &position);

private:
  std::pair<tf2::Vector3, PathPoint> find(const tf2::Vector3 &position);
  PathCalculator path_calculator_;
  PathPoint previousPoint_;
};

} // end namespace aav_control

#endif

