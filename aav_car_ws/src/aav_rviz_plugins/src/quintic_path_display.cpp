#include "aav_rviz_plugins/quintic_path_display.h"

#include <aav_quintic/path_calculator.h>
#include <OgreManualObject.h>

namespace aav_rviz_plugins
{

QuinticPathDisplay::QuinticPathDisplay()
{
  manual_object_ = NULL;
  color_property_ = new rviz::ColorProperty("Color", QColor(25, 255, 0),
      "The color of the path.", this);
  alpha_property_ = new rviz::FloatProperty("Alpha", 1.0,
      "The amount of transparency to apply to the path.", this);
}

QuinticPathDisplay::~QuinticPathDisplay()
{
  if (manual_object_)
  {
    manual_object_->clear();
    scene_manager_->destroyManualObject(manual_object_);
  }
}

void QuinticPathDisplay::onInitialize()
{
  MFDClass::onInitialize();
  manual_object_ = scene_manager_->createManualObject();
  manual_object_->setDynamic(true);
  scene_node_->attachObject(manual_object_);
}

void QuinticPathDisplay::processMessage(const aav_msgs::DoQuinticPathActionGoal::ConstPtr& goal)
{
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(goal->header, position, orientation))
  {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
        goal->header.frame_id.c_str(), qPrintable(fixed_frame_));
  }

  Ogre::Matrix4 transform(orientation);
  transform.setTrans(position);

  Ogre::ColourValue color = color_property_->getOgreColor();
  color.a = alpha_property_->getFloat();

  const aav_msgs::QuinticPath &path = goal->goal.path;
  aav_quintic::PathCalculator path_calculator(path);
  manual_object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);
  for (unsigned int i = 0; i < path_calculator.numSegments(); i++)
  {
    for (double t = 0; t < 1.0; t += .1)
    {
      tf2::Vector3 tf2v = path_calculator.calculate(i, t);
      fprintf(stderr, "%f, %f, %f\n", tf2v.x(), tf2v.y(), tf2v.z());
      Ogre::Vector3 ogrev((float) tf2v.x(), (float) tf2v.y(), (float) tf2v.z());
      manual_object_->position(transform * ogrev);
      manual_object_->colour(color);
    }
  }
  manual_object_->end();
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aav_rviz_plugins::QuinticPathDisplay, rviz::Display)

