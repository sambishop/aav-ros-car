#ifndef _QUINTIC_CONTROL_H_
#define _QUINTIC_CONTROL_H_

#include <boost/thread/mutex.hpp>

#include "aav_control/DoQuinticPathAction.h"
#include "nav_msgs/Odometry.h"

namespace aav_control {
  class QuinticControl {
  public:
    void updateGoal(const aav_control::DoQuinticPathGoalConstPtr &goal);
    void updateOdometry(const nav_msgs::Odometry::ConstPtr &odometry);

  private:
    const aav_msgs::QuinticPath *getPathFromGoal();
    boost::mutex goalMutex;
    DoQuinticPathGoalConstPtr goal;
  };
}

#endif

