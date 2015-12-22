#ifndef AAV_CONTROL_PID_H
#define AAV_CONTROL_PID_H

#include <ros/node_handle.h>

namespace aav_control
{

class Pid
{
public:
  Pid(ros::NodeHandle nh);
  void setSetpoint(double setpoint);
  double update(double value);

private:
  const double p_gain_;
  const double i_gain_;
  const double d_gain_;
  const double i_min_;
  const double i_max_;
  double setpoint_;
  double i_state_;
  double d_state_;
};

} // end namespace aav_control

#endif

