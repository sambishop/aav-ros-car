#include <aav_control/quintic_control.h>
#include <aav_msgs/DoQuinticPathAction.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, ROS_PACKAGE_NAME);
  ros::NodeHandle node;

  ros::Publisher cmd_pub = node.advertise<ackermann_msgs::AckermannDriveStamped>(
      "ackermann_cmd",
      1000
    );
  ros::Publisher cte_pub = node.advertise<std_msgs::Float64>(
      "cte",
      1000
    );
  aav_control::QuinticControl control(&cmd_pub, &cte_pub);
  ros::Subscriber sub = node.subscribe(
      "odometry/filtered",
      1000,
      &aav_control::QuinticControl::updateOdometry,
      &control
    );

  actionlib::SimpleActionServer<aav_msgs::DoQuinticPathAction> server(
      node,
      "control",
      boost::bind(&aav_control::QuinticControl::updateGoal, &control, _1),
      false
    );
  server.start();

  ros::spin();
  return 0;
}

