#include "aav_msgs/DoQuinticPathAction.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "actionlib/server/simple_action_server.h"
#include "ros/ros.h"
#include "aav_control/QuinticControl.h"

using namespace aav_control;
using namespace aav_msgs;
using namespace ackermann_msgs;
using namespace actionlib;
using namespace ros;

int main(int argc, char **argv) {
  init(argc, argv, "aav_control");
  /*
  NodeHandle node;

  Publisher pub = node.advertise<AckermannDriveStamped>(
      "ackermann_cmd",
      1000
    );
  QuinticControl control(&pub);
  Subscriber sub = node.subscribe(
      "odometry/filtered",
      1000,
      &QuinticControl::updateOdometry,
      &control
    );
  aav_msgs::PlanningPoint pp;
  pp.acceleration = 1.0;
  aav_msgs::QuinticPathSegment segment;
  segment.points.push_back(pp);
  segment.x_segment.P0 = 0.0;
  segment.x_segment.P1 = 1.0;
  segment.x_segment.P2 = 2.0;
  segment.x_segment.P3 = 8.0;
  segment.x_segment.P4 = 9.0;
  segment.x_segment.P5 = 10.0;
  aav_msgs::DoQuinticPathGoal goal;
  goal.path.segments.push_back(segment);
  control.updateGoal(aav_msgs::DoQuinticPathGoalConstPtr(&goal));

  SimpleActionServer<DoQuinticPathAction> server(
      node,
      "control",
      boost::bind(&QuinticControl::updateGoal, &control, _1),
      false
    );
  server.start();
   */

  spin();
  return 0;
}

