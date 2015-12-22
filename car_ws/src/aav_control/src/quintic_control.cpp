#include "aav_control/quintic_control.h"
#include "aav_control/cte_calculator.h"
#include "ackermann_msgs/AckermannDriveStamped.h"

#include <stdio.h>

namespace aav_control
{

QuinticControl::QuinticControl(ros::Publisher *publisher)
    : publisher_(publisher),
      steering_pid_(ros::NodeHandle("~steering_pid")),
      speed_pid_(ros::NodeHandle("~speed_pid")),
      speed_cmd_(0)
{
}

void QuinticControl::updateGoal(aav_msgs::DoQuinticPathGoalConstPtr goal)
{
  goal_mutex_.lock();
  goal_ = goal;
  goal_mutex_.unlock();
}

void QuinticControl::updateOdometry(nav_msgs::Odometry::ConstPtr odometry) {
  aav_msgs::QuinticPath path;
  goal_mutex_.lock();
  if (goal_)
    path = goal_->path;
  goal_mutex_.unlock();
  if (path.segments.empty())
    return;

  ackermann_msgs::AckermannDriveStamped msg;
  speed_pid_.setSetpoint(.2);

  const geometry_msgs::Point &point = odometry->pose.pose.position;
  tf2::Vector3 position(point.x, point.y, point.z);
  aav_control::CteCalculator calculator(path);
  double cte = calculator.calculate(position);
  msg.drive.steering_angle = std::isnan(cte) ? 0 : -steering_pid_.update(cte);

  double x_vel = odometry->twist.twist.linear.x;
  double y_vel = odometry->twist.twist.linear.y;
  double speed = sqrt(x_vel * x_vel + y_vel * y_vel); 
  if (std::isnan(cte))
    speed_cmd_ = 0;
  else
    speed_cmd_ += speed_pid_.update(speed);
  msg.drive.speed = speed_cmd_;

  publisher_->publish(msg);
  fprintf(stderr, "[x=%f, y=%f, z=%f], cte=%f, [x_vel=%f, y_vel=%f, speed=%f]\n",
      point.x, point.y, point.z, cte, x_vel, y_vel, speed);
}

} // end namespace aav_control

