#include "aav_control/spline_segment_calculator.h"

namespace aav_control
{

SplineSegmentCalculator::SplineSegmentCalculator(const aav_msgs::QuinticSplineSegment &segment)
  : P0_(segment.P0), P1_(segment.P1), P2_(segment.P2), P3_(segment.P3), P4_(segment.P4), P5_(segment.P5)
{}

double SplineSegmentCalculator::calculate(double t_1) const
{
  // The code below is an implementation of this function:
  // S(t) = (1-t)^5*P0 + 5*(1-t)^4*t*P1 + 10*(1-t)^3*t^2*P2
  //        + 10*(1-t)^2*t^3*P3 + 5(1-t)*t^4*P4 + t^5*P5
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
  // The code below is an implementation of this function:
  // S'(t) = - 5*P0*    (1-t)^4 +  5*P1*    (1-t)^4
  //        - 20*P1*t*  (1-t)^3
  //        - 30*P2*t^2*(1-t)^2 + 20*P2*t*  (1-t)^3
  //        - 20*P3*t^3*(1-t)   + 30*P3*t^2*(1-t)^2
  //         - 5*P4*t^4         + 20*P4*t^3*(1-t)
  //                             + 5*P5*t^4
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

} // end namespace aav_control

