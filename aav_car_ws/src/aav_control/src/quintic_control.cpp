#include "aav_control/quintic_control.h"

namespace aav_control
{

QuinticControl::QuinticControl(ros::Publisher *publisher)
    : publisher_(publisher),
      steering_pid_(.5, .1, .1, -4, 4),
      speed_pid_(.1, .1, .1, -4, 4),
      speed_cmd_(0)
{
}

} // end namespace aav_control

