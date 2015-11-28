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

Gamepad *openGamepad(const char *devicePath) {
  int fd = open(devicePath, O_RDONLY);
  if (fd == -1) {
    ROS_FATAL("'%s' attempting to open '%s' for reading\n",
        strerror(errno), devicePath);
    return NULL;
  }
  return new Gamepad(fd);
}

AckermannDrive readAckermannDrive(Gamepad *joystick) {
  Position position = joystick->readPosition();
  AckermannDrive drive;
  drive.speed = position.x;
  drive.steering_angle = position.y;
//  drive.speed = 4.0 / INT16_MAX * position.y;
//  drive.steering_angle = (M_PI / 4) / INT16_MAX * position.x;
//  drive.steering_angle_velocity = M_PI / INT16_MAX * position.x;
  return drive;
}

int main(int argc, char **argv) {
  //ros::init(argc, argv, ROS_PACKAGE_NAME);
  //ros::NodeHandle node;
  //Publisher publisher = node.advertise<AckermannDriveStamped>("/ackermann_vehicle/ackermann_cmd", 1000);
  Gamepad *joystick = openGamepad("/dev/input/js0");
  if (joystick == NULL) {
    return 2;
  }

  //while (ros::ok()) {
  while (1) {
    AckermannDrive drive = readAckermannDrive(joystick);
    ROS_INFO("speed = %f, steering_angle = %f\n",
        drive.speed, drive.steering_angle);
    //publisher.publish(msg);
    //ros::spinOnce();
  }
  return 0;
}

