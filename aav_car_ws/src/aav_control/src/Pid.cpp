#include <algorithm>

#include "Pid.h"

using namespace aav_control;

Pid::Pid(double setPoint, double pGain, double iGain, double dGain,
    double iMin, double iMax)
  : setPoint(setPoint), pGain(pGain), iGain(iGain), dGain(dGain),
    iMin(iMin), iMax(iMax)
{}

double Pid::update(double value) {
  error = setPoint - value;

  double pValue = pGain * error;

  double dValue = dGain * (error - dState);
  dState = error;

  iState += error;
  iState = std::max(std::min(iState, iMax), iMin);
  double iValue = iState * iGain;

  return pValue + iValue + dValue;
}

