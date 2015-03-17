#ifndef _QUINTIC_CONTROL_H_
#define _QUINTIC_CONTROL_H_

#include "aav_control/DoQuinticPathAction.h"

namespace aav_control {

class QuinticControl {
  public:
    void callback(const aav_control::DoQuinticPathGoalConstPtr &goal);
};

} // namespace aav_control

#endif

