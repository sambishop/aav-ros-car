#ifndef AAV_CONTROL_CROSS_TRACK_ERROR_CALCULATOR_H
#define	AAV_CONTROL_CROSS_TRACK_ERROR_CALCULATOR_H

#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Vector3.h>
#include <utility>

#include "path_point.h"

namespace aav_control
{

class CrossTrackErrorCalculator
{
public:
  double calculate(nav_msgs::Odometry::ConstPtr &odometry);

private:
  std::pair<tf2::Vector3, aav_control::PathPoint> calculate();
  PathPoint previousLocation;
};

} // end namespace aav_control

#endif

