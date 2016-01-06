#ifndef AAV_CONTROL_GAZEBO_STATE_FORWARDER_H
#define AAV_CONTROL_GAZEBO_STATE_FORWARDER_H

#include <gazebo_msgs/ModelStates.h>
#include <string>

#include "aav_control/quintic_control.h"

namespace aav_control
{

class GazeboStateForwarder
{
public:
  GazeboStateForwarder(aav_control::QuinticControl &control, std::string model_name);
  void forwardState(gazebo_msgs::ModelStatesConstPtr model_states);

private:
  aav_control::QuinticControl &control_;
  std::string model_name_;
};

}

#endif

