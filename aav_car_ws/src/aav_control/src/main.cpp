#include "aav_control/DoQuinticPathAction.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "actionlib/server/simple_action_server.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "QuinticControl.h"

using namespace aav_control;
using namespace ackermann_msgs;
using namespace actionlib;
using namespace ros;

int main(int argc, char **argv)
{
  init(argc, argv, "aav_control");
  NodeHandle node;
  QuinticControl control;

  Publisher pub = node.advertise<AckermannDriveStamped>("ackermann_cmd",
      1000);
  /*
  Subscriber sub = node.subscribe(
      "odometry/filtered",
      1000,
      boost::bind(control, QuinticControl::callback);
  */

  SimpleActionServer<DoQuinticPathAction> server(
      node,
      "aav_control", 
      boost::bind(&QuinticControl::callback, &control, _1),
      false
    );
  server.start();
  spin();
  return 0;
}

