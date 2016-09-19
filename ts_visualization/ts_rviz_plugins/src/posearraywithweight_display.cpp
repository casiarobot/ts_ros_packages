#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreColourValue.h>

#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/validate_floats.h"

#include "posearraywithweight_display.h"

namespace ts_rviz_plugins
{

PoseArrayWithWeight::PoseArrayWithWeight()
  : manual_object_( NULL )
{
//  color_property_ = new rviz::ColorProperty( "Color", QColor( 255, 25, 0 ), "Color to draw the arrows.", this );
  length_property_ = new rviz::FloatProperty( "Arrow Length", 0.3, "Length of the arrows.", this );
}

PoseArrayWithWeight::~PoseArrayWithWeight()
{
  if ( initialized() )
  {
    scene_manager_->destroyManualObject( manual_object_ );
  }
}

void PoseArrayWithWeight::onInitialize()
{
  MFDClass::onInitialize();
  manual_object_ = scene_manager_->createManualObject();
  manual_object_->setDynamic( true );
  scene_node_->attachObject( manual_object_ );
}

bool validateFloats( const ts_msgs::PoseArrayWithWeight& msg )
{
  return rviz::validateFloats( msg.poses );
}

void PoseArrayWithWeight::processMessage( const ts_msgs::PoseArrayWithWeight::ConstPtr& msg )
{
  if( !validateFloats( *msg ))
  {
    setStatus( rviz::StatusProperty::Error, "Topic", "Message contained invalid floating point values (nans or infs)" );
    return;
  }

  manual_object_->clear();

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if( !context_->getFrameManager()->getTransform( msg->header, position, orientation ))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
  }

  scene_node_->setPosition( position );
  scene_node_->setOrientation( orientation );

  manual_object_->clear();

//  Ogre::ColourValue color = color_property_->getOgreColor();
  Ogre::ColourValue color;
  float length = length_property_->getFloat();
  size_t num_poses = msg->poses.size();
  manual_object_->estimateVertexCount( num_poses * 6 );
  manual_object_->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST );
  for( size_t i=0; i < num_poses; ++i )
  {
    color.setHSB((1-msg->weight[i])*240/360,1.0,1.0);
    Ogre::Vector3 pos( msg->poses[i].position.x,
                       msg->poses[i].position.y,
                       msg->poses[i].position.z );
    Ogre::Quaternion orient( msg->poses[i].orientation.w,
                             msg->poses[i].orientation.x,
                             msg->poses[i].orientation.y,
                             msg->poses[i].orientation.z );
    // orient here is not normalized, so the scale of the quaternion
    // will affect the scale of the arrow.

    Ogre::Vector3 vertices[6];
    vertices[0] = pos; // back of arrow
    vertices[1] = pos + orient * Ogre::Vector3( length, 0, 0 ); // tip of arrow
    vertices[2] = vertices[ 1 ];
    vertices[3] = pos + orient * Ogre::Vector3( 0.75*length, 0.2*length, 0 );
    vertices[4] = vertices[ 1 ];
    vertices[5] = pos + orient * Ogre::Vector3( 0.75*length, -0.2*length, 0 );

    for( int i = 0; i < 6; ++i )
    {
      manual_object_->position( vertices[i] );
      manual_object_->colour( color );
    }
  }
  manual_object_->end();

  context_->queueRender();
}

void PoseArrayWithWeight::reset()
{
  MFDClass::reset();
  if( manual_object_ )
  {
    manual_object_->clear();
  }
}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( ts_rviz_plugins::PoseArrayWithWeight, rviz::Display )
