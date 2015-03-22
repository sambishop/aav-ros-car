#ifndef _PID_H_
#define _PID_H_

namespace aav_control {
  class Pid {
  public:
    Pid(double setPoint, double pGain, double iGain, double dGain,
        double iMin, double iMax);
    double update(double value);

  private:
    const double setPoint;
    const double pGain;
    const double iGain;
    const double dGain;
    const double iMin;
    const double iMax;
    double iState;
    double dState;
    double error;
  };
}

#endif

