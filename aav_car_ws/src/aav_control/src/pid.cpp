#include "aav_control/pid.h"

#include <algorithm>

namespace aav_control
{

Pid::Pid(double p_gain, double i_gain, double d_gain, double i_min, double i_max)
  : p_gain_(p_gain), i_gain_(i_gain), d_gain_(d_gain),
    i_min_(i_min), i_max_(i_max)
{}

void Pid::setSetpoint(double setpoint)
{
  setpoint_ = setpoint;
}

double Pid::update(double value)
{
  error_ = setpoint_ - value;

  double p_value = p_gain_ * error_;

  /*
  i_state_ += error_;
  i_state_ = std::max(std::min(i_state_, i_max_), i_min_);
  double i_value = i_state_ * i_gain_;

  double d_value = d_gain_ * (error_ - d_state_);
  d_state_ = error_;
   */

  return p_value; /* + i_value + d_value;*/
}

} // end namespace aav_control

