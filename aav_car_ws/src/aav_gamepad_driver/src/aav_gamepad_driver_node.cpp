#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

#include "ackermann_msgs/AckermannDrive.h"
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
    }
    return new Joystick(fd);
}

AckermannDrive readAckermannMsg(Joystick *joystick)
{
    Event event = joystick->readEvent();

    AckermannDrive msg;
    return msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, ROS_PACKAGE_NAME);
    NodeHandle node;
    Publisher publisher = node.advertise<AckermannDrive>("msg", 10);
    Joystick *joystick = openJoystick("/dev/input/js0");
    while (ros::ok()) {
        AckermannDrive msg = readAckermannMsg(joystick);
        publisher.publish(msg);
        ros::spinOnce();
    }
    return 0;
}

