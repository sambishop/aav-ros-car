#ifndef _QUINTIC_CONTROL_H_
#define _QUINTIC_CONTROL_H_

#include "aav_control/DoQuinticPathAction.h"
#include "nav_msgs/Odometry.h"

namespace aav_control {

class QuinticControl {
  public:
    void updateGoal(const aav_control::DoQuinticPathGoalConstPtr &goal);
    void updateOdometry(const nav_msgs::Odometry::ConstPtr &odometry);

  private:
    DoQuinticPathGoalConstPtr goal;
};

} // namespace aav_control

#endif

