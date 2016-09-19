#ifndef RVIZ_POSE_ARRAY_DISPLAY_H_
#define RVIZ_POSE_ARRAY_DISPLAY_H_

#include <geometry_msgs/PoseArray.h>
#include "ts_msgs/PoseArrayWithWeight.h"

#include "rviz/message_filter_display.h"

namespace Ogre
{
class ManualObject;
}
namespace rviz
{
class ColorProperty;
class FloatProperty;
}
namespace ts_rviz_plugins
{

class PoseArrayWithWeight: public rviz::MessageFilterDisplay<ts_msgs::PoseArrayWithWeight>
{
Q_OBJECT
public:
  PoseArrayWithWeight();
  virtual ~PoseArrayWithWeight();

protected:
  virtual void onInitialize();
  virtual void reset();
  virtual void processMessage( const ts_msgs::PoseArrayWithWeight::ConstPtr& msg );

private:
  Ogre::ManualObject* manual_object_;

//  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* length_property_;
};

} // namespace rviz

#endif /* RVIZ_POSE_ARRAY_DISPLAY_H_ */
