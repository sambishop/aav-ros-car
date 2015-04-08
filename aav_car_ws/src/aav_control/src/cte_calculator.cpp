#include "cte_calculator.h"

namespace aav_control
{

CteCalculator::CteCalculator(const aav_msgs::QuinticPath &path)
  : path_calculator_(path), previousPoint_(0, 0)
{
}

} // end namespace aav_control

