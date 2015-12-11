#include "aav_control/pid.h"

#include <algorithm>

namespace aav_control
{

Pid::Pid(ros::NodeHandle nh)
  : p_gain_(nh.param("p_gain", 0)),
    i_gain_(nh.param("i_gain", 0)),
    d_gain_(nh.param("d_gain", 0)),
    i_min_(nh.param("i_min", 0)),
    i_max_(nh.param("i_max", 0)),
    setpoint_(0),
    i_state_(0),
    d_state_(0)
{}

void Pid::setSetpoint(double setpoint)
{
  setpoint_ = setpoint;
}

double Pid::update(double value)
{
  double error = setpoint_ - value;

  double p_value = p_gain_ * error;

  i_state_ += error;
  i_state_ = std::max(std::min(i_state_, i_max_), i_min_);
  double i_value = i_state_ * i_gain_;

  double d_value = d_gain_ * (error - d_state_);
  d_state_ = error;

  return p_value + i_value + d_value;
}

} // end namespace aav_control

