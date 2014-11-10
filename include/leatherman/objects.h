#ifndef _OBJECTS_
#define _OBJECTS_

#include <ros/console.h>
#include <arm_navigation_msgs/CollisionObject.h>
#include <visualization_msgs/MarkerArray.h>
#include <shape_msgs/SolidPrimitive.h>

namespace leatherman
{

  bool getCollisionObjects(std::string filename, std::string frame_id, bool transform_objects, geometry_msgs::Pose &object_transform, std::vector<arm_navigation_msgs::CollisionObject> &collision_objects, visualization_msgs::MarkerArray &object_markers);

  void convertShapeToSolidPrimitive(const arm_navigation_msgs::Shape &shape, shape_msgs::SolidPrimitive &solid);

}

#endif
