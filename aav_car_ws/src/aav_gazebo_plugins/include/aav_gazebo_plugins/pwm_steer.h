#include <boost/bind.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace aav_gazebo_plugins {
  class PwmSteer : public gazebo::ModelPlugin {
    public:
      void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);

    private:
      void OnUpdate(const gazebo::common::UpdateInfo &);
      gazebo::physics::ModelPtr model;
      sdf::ElementPtr sdfElement;
      gazebo::event::ConnectionPtr updateConnection;
  };
}

