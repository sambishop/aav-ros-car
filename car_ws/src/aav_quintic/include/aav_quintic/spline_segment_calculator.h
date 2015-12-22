#ifndef AAV_QUINTIC_SPLINE_SEGMENT_CALCULATOR_H
#define	AAV_QUINTIC_SPLINE_SEGMENT_CALCULATOR_H

#include <aav_msgs/QuinticSplineSegment.h>

namespace aav_quintic
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

} // end namespace aav_quintic

#endif

