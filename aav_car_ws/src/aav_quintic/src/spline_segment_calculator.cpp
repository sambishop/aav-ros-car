#include "aav_quintic/spline_segment_calculator.h"

namespace aav_quintic
{

SplineSegmentCalculator::SplineSegmentCalculator(const aav_msgs::QuinticSplineSegment &segment)
  : P0_(segment.P0), P1_(segment.P1), P2_(segment.P2), P3_(segment.P3), P4_(segment.P4), P5_(segment.P5)
{}

// The code below uses these two patterns for variable names:
// t_n = t raised to the nth power
// _t_n = 1-t raised to the nth power

double SplineSegmentCalculator::calculate(double t_1) const
{
  double t_2 = t_1 * t_1;
  double t_3 = t_2 * t_1;
  double t_4 = t_2 * t_2;
  double t_5 = t_4 * t_1;

  double _t_1 = 1.0 - t_1;
  double _t_2 = _t_1 * _t_1;
  double _t_3 = _t_2 * _t_1;
  double _t_4 = _t_2 * _t_2;
  double _t_5 = _t_4 * _t_1;

  return     _t_5 *       P0_
      +  5 * _t_4 * t_1 * P1_
      + 10 * _t_3 * t_2 * P2_
      + 10 * _t_2 * t_3 * P3_
      +  5 * _t_1 * t_4 * P4_
      +             t_5 * P5_;
}

double SplineSegmentCalculator::calculate1stDerivative(double t_1) const
{
  double t_2 = t_1 * t_1;
  double t_3 = t_2 * t_1;
  double t_4 = t_2 * t_2;

  double _t_1 = 1.0 - t_1;
  double _t_2 = _t_1 * _t_1;
  double _t_3 = _t_2 * _t_1;
  double _t_4 = _t_2 * _t_2;

  return - 5 * P0_       * _t_4 +  5 * P1_       * _t_4
        - 20 * P1_ * t_1 * _t_3
        - 30 * P2_ * t_2 * _t_2 + 20 * P2_ * t_1 * _t_3
        - 20 * P3_ * t_3 * _t_1 + 30 * P3_ * t_2 * _t_2
        -  5 * P4_ * t_4        + 20 * P4_ * t_3 * _t_1
                                +  5 * P5_ * t_4;
}

} // end namespace aav_quintic

