#ifndef AAV_CONTROL_PID_H
#define AAV_CONTROL_PID_H

namespace aav_control
{

class Pid
{
public:
  Pid(double p_gain, double i_gain, double d_gain, double i_min, double i_max);
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
  double error_;
};

} // end namespace aav_control

#endif

