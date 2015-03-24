#include "QuinticControl.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "aav_control2");
  ros::NodeHandle node;
  aav_control::QuinticControl control;
  ros::Subscriber subscriber = node.subscribe(
      "odometry/filtered",
      1000,
      &aav_control::QuinticControl::updateOdometry,
      &control
    );
  ros::spin();
  return 0;
}

