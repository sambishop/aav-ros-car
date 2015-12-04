#include "aav_control/quintic_control.h"
#include "aav_control/cte_calculator.h"
#include "ackermann_msgs/AckermannDriveStamped.h"

namespace aav_control
{

QuinticControl::QuinticControl(ros::Publisher *publisher)
    : publisher_(publisher),
      steering_pid_(.5, .1, .1, -4, 4),
      speed_pid_(.1, .1, .1, -4, 4),
      speed_cmd_(0)
{
}

void QuinticControl::updateGoal(aav_msgs::DoQuinticPathGoalConstPtr &goal)
{
  goal_mutex_.lock();
  goal_ = goal;
  goal_mutex_.unlock();
}

void QuinticControl::updateOdometry(nav_msgs::Odometry::ConstPtr &odometry) {
  aav_msgs::QuinticPath path;
  goal_mutex_.lock();
  if (goal_)
    path = goal_->path;
  goal_mutex_.unlock();
  if (path.segments.empty())
    return;

  const geometry_msgs::Point &point = odometry->pose.pose.position;
  tf2::Vector3 position(point.x, point.y, point.z);
  aav_control::CteCalculator calculator(path);
  double cte = calculator.calculate(position);

  double x_vel = odometry->twist.twist.linear.x;
  double y_vel = odometry->twist.twist.linear.y;
  double speed = sqrt(x_vel * x_vel + y_vel * y_vel); 
  speed_pid_.setSetpoint(.2);

  ackermann_msgs::AckermannDriveStamped msg;
  msg.drive.steering_angle = -steering_pid_.update(cte);
  speed_cmd_ += speed_pid_.update(speed);
  msg.drive.speed = speed_cmd_;
  publisher_->publish(msg);
}

} // end namespace aav_control

