#include "aav_control/quintic_control.h"
#include "aav_msgs/DoQuinticPathAction.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "actionlib/server/simple_action_server.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, ROS_PACKAGE_NAME);
  ros::NodeHandle node;

  ros::Publisher pub = node.advertise<ackermann_msgs::AckermannDriveStamped>(
      "ackermann_cmd",
      1000
    );
  aav_control::QuinticControl control(&pub);
  ros::Subscriber sub = node.subscribe(
      "odometry/filtered",
      1000,
      &aav_control::QuinticControl::updateOdometry,
      &control
    );
  aav_msgs::PlanningPoint pp;
  pp.acceleration = 1.0;
  aav_msgs::QuinticPathSegment segment;
  segment.points.push_back(pp);
  segment.y_segment.P0 = 0.0;
  segment.y_segment.P1 = 1.0;
  segment.y_segment.P2 = 2.0;
  segment.y_segment.P3 = 8.0;
  segment.y_segment.P4 = 9.0;
  segment.y_segment.P5 = 10.0;
  aav_msgs::DoQuinticPathGoal goal;
  goal.path.segments.push_back(segment);
  aav_msgs::DoQuinticPathGoalConstPtr goal_ptr(&goal);
  control.updateGoal(goal_ptr);

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

