#ifndef _PID_H_
#define _PID_H_

namespace aav_control
{
  class Pid
  {
  public:
    Pid(double p_gain, double i_gain, double d_gain, double i_min, double i_max);
    void setSetpoint(double setpoint);
    double update(double value);

  private:
    double setpoint_;
    const double p_gain_;
    const double i_gain_;
    const double d_gain_;
    const double i_min_;
    const double i_max_;
    double i_state_;
    double d_state_;
    double error_;
  };
}

#endif

