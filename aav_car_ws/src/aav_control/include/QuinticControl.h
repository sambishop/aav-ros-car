#ifndef _QUINTIC_CONTROL_H_
#define _QUINTIC_CONTROL_H_

#include <boost/thread/mutex.hpp>

#include "aav_msgs/DoQuinticPathAction.h"
#include "nav_msgs/Odometry.h"
#include "Pid.h"
#include "ros/ros.h"

namespace aav_control {
  class QuinticControl {
  public:
    QuinticControl(ros::Publisher *pub);
    void updateGoal(const aav_msgs::DoQuinticPathGoalConstPtr &goal);
    void updateOdometry(const nav_msgs::Odometry::ConstPtr &odometry);

  private:
    ros::Publisher *pub;
    const aav_msgs::QuinticPath *getPathFromGoal();
    boost::mutex goalMutex;
    aav_msgs::DoQuinticPathGoalConstPtr goal;
    Pid steeringPid;
    Pid speedPid;
  };
}

#endif

