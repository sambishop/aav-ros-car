#ifndef AAV_CONTROL_CTE_CALCULATOR_H
#define	AAV_CONTROL_CTE_CALCULATOR_H

#include <aav_msgs/QuinticPath.h>
#include <tf2/LinearMath/Vector3.h>
#include <vector>

#include "path_segment_calculator.h"

namespace aav_control
{

class CteCalculator
{
public:
  CteCalculator(const aav_msgs::QuinticPath &path);
  ~CteCalculator();
  double calculate(const tf2::Vector3 &position);

private:
  double calculateDistanceMeasure(double t, void *that);
  const aav_msgs::QuinticPath &path_;
  const tf2::Vector3 *position_;
  unsigned int segment_index_;
  double prev_t_;
  std::vector<const PathSegmentCalculator *> calculators_;
};

} // end namespace aav_control

#endif

