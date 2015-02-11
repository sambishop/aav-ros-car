#include <ros/ros.h>

#include "aav_gazebo_plugins/pwm_steer.h"

namespace aav_gazebo_plugins {
  void PwmSteer::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
  {
    ROS_INFO_STREAM_NAMED("pwm_steer", "loading pwm_steer plugin");
    if (!model) {
      ROS_ERROR_STREAM_NAMED("pwm_steer", "model is null");
      return;
    }
    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM_NAMED("pwm_steer",
          "a ROS node for Gazebo has not been initialized");
      return;
    }
    this->model = model;
    this->sdfElement = sdf;
    this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&PwmSteer::OnUpdate, this, _1));
  }

  void PwmSteer::OnUpdate(const gazebo::common::UpdateInfo &)
  {
    //model->SetLinearVel(gazebo::math::Vector3(.03, 0, 0));
    model->GetLink("back_left_wheel")->SetTorque(gazebo::math::Vector3(0, 3, 0));
    model->GetLink("back_right_wheel")->SetTorque(gazebo::math::Vector3(0, 3, 0));
    model->GetLink("front_left_wheel")->SetTorque(gazebo::math::Vector3(0, 3, 0));
    model->GetLink("front_right_wheel")->SetTorque(gazebo::math::Vector3(0, 3, 0));
  }

  GZ_REGISTER_MODEL_PLUGIN(PwmSteer)
}

