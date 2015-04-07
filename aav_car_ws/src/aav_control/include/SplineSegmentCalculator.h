#ifndef AAV_CONTROL_SPLINESEGMENTCALCULATOR_H
#define	AAV_CONTROL_SPLINESEGMENTCALCULATOR_H

#include <aav_msgs/QuinticSplineSegment.h>

namespace aav_control
{

class SplineSegmentCalculator
{
public:
  SplineSegmentCalculator(const aav_msgs::QuinticSplineSegment &segment);
  double calculate(double t) const;
  double calculate1stDerivative(double t) const;

private:
  const double P0_;
  const double P1_;
  const double P2_;
  const double P3_;
  const double P4_;
  const double P5_;
};

} // end namespace aav_control

#endif

