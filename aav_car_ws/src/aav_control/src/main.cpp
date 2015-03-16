#include "aav_control/DoQuinticPathAction.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "actionlib/server/simple_action_server.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

using namespace aav_control;
using namespace ackermann_msgs;
using namespace actionlib;
using namespace ros;

void callback(const DoQuinticPathGoalConstPtr &goal)
{
}

int main(int argc, char **argv)
{
  init(argc, argv, "aav_control");
  NodeHandle node;

  Publisher pub = node.advertise<AckermannDriveStamped>("quintic_path", 1000);
  Subscriber sub = node.subscribe("odometry/filtered", 1000, callback);

  SimpleActionServer<DoQuinticPathAction> server(
      node,
      "aav_control", 
      boost::bind(callback, _1),
      false
    );
  server.start();
  spin();
  return 0;
}

