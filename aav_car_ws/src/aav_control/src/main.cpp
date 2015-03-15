#include "aav_control/DoQuinticPathAction.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "actionlib/client/simple_action_client.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

using namespace aav_control;
using namespace ackermann_msgs;
using namespace actionlib;
using namespace ros;

typedef SimpleActionClient<DoQuinticPathAction> Client;

void callback(const nav_msgs::Odometry::ConstPtr& msg)
{
}

int main(int argc, char **argv)
{
  init(argc, argv, "aav_control");
  NodeHandle node;

  Publisher pub = node.advertise<AckermannDriveStamped>("quintic_path", 1000);
  Subscriber sub = node.subscribe("odometry/filtered", 1000, callback);

  return 0;
}

