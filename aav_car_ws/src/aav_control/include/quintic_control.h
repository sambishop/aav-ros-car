#ifndef AAV_CONTROL_QUINTIC_CONTROL_H
#define AAV_CONTROL_QUINTIC_CONTROL_H

#include <aav_msgs/DoQuinticPathAction.h>
#include <boost/thread/mutex.hpp>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include "pid.h"

namespace aav_control
{

class QuinticControl
{
public:
  QuinticControl(ros::Publisher *publisher);
  void updateGoal(const aav_msgs::DoQuinticPathGoalConstPtr &goal);
  void updateOdometry(const nav_msgs::Odometry::ConstPtr &odometry);

private:
  const aav_msgs::QuinticPath *getPathFromGoal();
  ros::Publisher *publisher_;
  Pid steering_pid_;
  Pid speed_pid_;
  double speed_cmd_;
  boost::mutex goal_mutex_;
  aav_msgs::DoQuinticPathGoalConstPtr goal_;
};

} // end namespace aav_control

#endif

