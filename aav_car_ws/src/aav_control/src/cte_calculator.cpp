#include "cte_calculator.h"

#include <limits>

namespace aav_control
{

CteCalculator::CteCalculator(const aav_msgs::QuinticPath &path)
  : path_calculator_(path), previousPoint_(0, 0)
{}

double CteCalculator::calculate(const tf2::Vector3 &position)
{
  return std::numeric_limits<double>::quiet_NaN();
}

} // end namespace aav_control

