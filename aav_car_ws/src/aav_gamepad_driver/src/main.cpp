#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

#include "ackermann_msgs/AckermannDrive.h"
#include "ros/ros.h"

#include "Gamepad.h"
using aav_gamepad_driver::Gamepad;
using aav_gamepad_driver::Position;
using ackermann_msgs::AckermannDrive;

#define MAX_SPEED_METERS_PER_SECOND 2
#define MAX_STEERING_ANGLE_DEGREES 30

Gamepad *openGamepad(const char *path) {
  int fd = open(path, O_RDONLY);
  if (fd == -1) {
    ROS_FATAL("'%s' attempting to open '%s' for reading",
        strerror(errno), path);
    return NULL;
  }
  return new Gamepad(fd);
}

AckermannDrive positionToAckermann(Position pos) {
  AckermannDrive drive;

  drive.speed = pos.y / 32767.0 * MAX_SPEED_METERS_PER_SECOND;

  // The steering angle should be in radians, with zero as straight
  // and positive to the left.
  float multiplier = MAX_STEERING_ANGLE_DEGREES / 90.0 * M_PI / 2;
  drive.steering_angle = -pos.x / 32767 * multiplier;

  return drive;
}

int main(int argc, char **argv) {
  //ros::init(argc, argv, ROS_PACKAGE_NAME);
  //ros::NodeHandle node;
  //Publisher publisher = node.advertise<AckermannDriveStamped>("/ackermann_vehicle/ackermann_cmd", 1000);
  Gamepad *gamepad = openGamepad("/dev/input/js0");
  if (gamepad == NULL) {
    return 2;
  }

  //while (ros::ok()) {
  while (1) {
    Position pos = gamepad->readPosition();
    AckermannDrive drive = positionToAckermann(pos);
    ROS_INFO("speed = %f, steering_angle = %f",
        drive.speed, drive.steering_angle);
    //publisher.publish(msg);
    //ros::spinOnce();
  }
  return 0;
}

