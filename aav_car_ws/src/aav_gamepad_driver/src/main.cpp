#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

#include "ackermann_msgs/AckermannDriveStamped.h"
#include "ros/ros.h"

#include "Gamepad.h"

using namespace aav_gamepad_driver;
using namespace ackermann_msgs;
using namespace ros;

#define MAX_SPEED_METERS_PER_SECOND 4
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

AckermannDriveStamped positionToAckermann(Position pos) {
  AckermannDriveStamped msg;

  msg.drive.speed = pos.y / 32767.0 * MAX_SPEED_METERS_PER_SECOND;

  // The steering angle is in radians, with zero as straight and
  // positive to the left.  Convert the x component of the stick
  // position to an angle, clamp to MAX_STEERING_ANGLE_DEGREES,
  // and then convert to radians.
  float steering_angle = -pos.x / 32767.0 * 90;
  steering_angle = fminf(steering_angle, MAX_STEERING_ANGLE_DEGREES);
  steering_angle = fmaxf(steering_angle, -MAX_STEERING_ANGLE_DEGREES);
  msg.drive.steering_angle = steering_angle / 180 * M_PI;

  return msg;
}

int main(int argc, char **argv) {
  init(argc, argv, ROS_PACKAGE_NAME);
  NodeHandle node;
  Publisher publisher = node.advertise<AckermannDriveStamped>(
      "ackermann_cmd",
      1000);
  Gamepad *gamepad = openGamepad("/dev/input/js0");
  if (gamepad == NULL) {
    return 2;
  }

  while (ok()) {
    Position pos = gamepad->readPosition();
    AckermannDriveStamped msg = positionToAckermann(pos);
    msg.header.stamp = Time::now();
    ROS_INFO("speed = %f, steering_angle = %f",
        msg.drive.speed, msg.drive.steering_angle);
    publisher.publish(msg);
    spinOnce();
  }
  return 0;
}

