#ifndef AAV_RVIZ_PLUGINS_QUINTIC_PATH_DISPLAY_H
#define AAV_RVIZ_PLUGINS_QUINTIC_PATH_DISPLAY_H

#include <aav_msgs/DoQuinticPathActionGoal.h>
#include <rviz/message_filter_display.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>

namespace aav_rviz_plugins
{

class QuinticPathDisplay : public rviz::MessageFilterDisplay<aav_msgs::DoQuinticPathActionGoal>
{
Q_OBJECT
public:
  QuinticPathDisplay();
  ~QuinticPathDisplay();

protected:
  void onInitialize();
  void processMessage(const aav_msgs::DoQuinticPathActionGoal::ConstPtr& goal);

private:
  Ogre::ManualObject* manual_object_;
  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* alpha_property_;
};

}

#endif

