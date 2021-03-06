#include <aav_control/quintic_control.h>
#include <aav_control/cte_calculator.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Float64.h>

#include <stdio.h>

namespace aav_control
{

QuinticControl::QuinticControl(ros::Publisher *cmd_pub, ros::Publisher *cte_pub)
    : cmd_pub_(cmd_pub),
      cte_pub_(cte_pub),
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
  tf2::Vector3 position(point.x, point.y, 0);
  aav_control::CteCalculator calculator(path);
  double cte = calculator.calculate(position);
  publishCte(cte);
  msg.drive.steering_angle = std::isnan(cte) ? 0 : -steering_pid_.update(cte);

  double x_vel = odometry->twist.twist.linear.x;
  double y_vel = odometry->twist.twist.linear.y;
  double speed = sqrt(x_vel * x_vel + y_vel * y_vel); 
  if (std::isnan(cte))
    speed_cmd_ = 0;
  else
    speed_cmd_ += speed_pid_.update(speed);
  msg.drive.speed = speed_cmd_;
  cmd_pub_->publish(msg);
}

void QuinticControl::publishCte(double cte)
{
  std_msgs::Float64 cte_msg;
  cte_msg.data = cte;
  cte_pub_->publish(cte_msg);
}

} // end namespace aav_control

