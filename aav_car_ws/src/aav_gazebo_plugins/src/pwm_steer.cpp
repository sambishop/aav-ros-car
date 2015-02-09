#include <ros/ros.h>

#include "aav_gazebo_plugins/pwm_steer.h"

namespace aav_gazebo_plugins {
  void PwmSteer::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf)
  {
    ROS_INFO_STREAM_NAMED("pwm_steer", "loading pwm_steer plugin");
    if (!parent) {
      ROS_ERROR_STREAM_NAMED("pwm_steer", "parent model is null");
      return;
    }
    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM_NAMED("pwm_steer",
          "a ROS node for Gazebo has not been initialized");
      return;
    }
    this->parentModel = parent;
    this->sdfElement = sdf;
    this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&PwmSteer::OnUpdate, this, _1));
  }

  void PwmSteer::OnUpdate(const gazebo::common::UpdateInfo &)
  {
    parentModel->SetLinearVel(gazebo::math::Vector3(.03, 0, 0));
  }

  GZ_REGISTER_MODEL_PLUGIN(PwmSteer)
}

