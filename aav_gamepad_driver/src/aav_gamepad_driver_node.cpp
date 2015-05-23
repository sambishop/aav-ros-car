#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

#include "ackermann_msgs/AckermannDriveStamped.h"
#include "Joystick.h"
#include "ros/ros.h"

using namespace ackermann_msgs;
using namespace ros;

Joystick *openJoystick(const char *devicePath)
{
  int fd = open(devicePath, O_RDONLY);
  if (fd == -1) {
    fprintf(stderr, "'%s' attempting to open '%s' for reading\n",
        strerror(errno), devicePath);
    exit(2);
  }
  return new Joystick(fd);
}

AckermannDriveStamped readAckermannMsg(Joystick *joystick)
{
  Position position = joystick->readPosition();
  AckermannDriveStamped msg;
  msg.drive.speed = -4.0 / 32767 * position.y;
  msg.drive.steering_angle = -(3.14159265359 / 4) / 32767 * position.x;
  msg.drive.steering_angle_velocity = -3.14159265359 / 32767 * position.x;
  return msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, ROS_PACKAGE_NAME);
  NodeHandle node;
  Publisher publisher = node.advertise<AckermannDriveStamped>("/ackermann_vehicle/ackermann_cmd", 1000);
  Joystick *joystick = openJoystick("/dev/input/js0");
  while (ros::ok()) {
    AckermannDriveStamped msg = readAckermannMsg(joystick);
    publisher.publish(msg);
    ros::spinOnce();
  }
  return 0;
}

