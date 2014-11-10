#include <iostream>
#include <cmath>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <leatherman/objects.h>
#include <leatherman/viz.h>
#include <leatherman/utils.h>
#include <leatherman/print.h>

namespace leatherman {

bool getCollisionObjects(std::string filename, std::string frame_id, bool transform_objects, geometry_msgs::Pose &object_transform, std::vector<arm_navigation_msgs::CollisionObject> &collision_objects, visualization_msgs::MarkerArray &object_markers)
{
  int num_obs;
  char sTemp[1024];
  visualization_msgs::Marker marker;
  std::vector<std::vector<double> > objects, object_colors;
  std::vector<double> object_transform_v;
  std::vector<std::string> object_ids;
  arm_navigation_msgs::CollisionObject object;
  collision_objects.clear();
  object_markers.markers.clear();
  leatherman::poseMsgToPoseVector(object_transform, object_transform_v);

  FILE* fCfg = fopen(filename.c_str(), "r");
  if(fCfg == NULL)
    return false;

  // get number of objects
  if(fscanf(fCfg,"%d",&num_obs) < 1)
    ROS_INFO("[objects] Parsed string has length < 1.(number of obstacles)\n");
  ROS_DEBUG("[objects] Parsing collision object file with %i objects.", num_obs);

  //get {x y z dimx dimy dimz red green blue alpha} for each object
  objects.resize(num_obs, std::vector<double>(6,0.0));
  object_colors.resize(num_obs, std::vector<double>(4,0.0));
  object_ids.clear();
  for (int i=0; i < num_obs; ++i)
  {
    if(fscanf(fCfg,"%s",sTemp) < 1)
      ROS_INFO("[objects] Parsed string has length < 1.\n");
    object_ids.push_back(sTemp);

    for(int j=0; j < 6; ++j)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        ROS_INFO("[objects] Parsed string has length < 1. (object parameters for %s)", object_ids.back().c_str());
      if(!feof(fCfg) && strlen(sTemp) != 0)
        objects[i][j] = atof(sTemp);
    }
    for(int j=0; j < 4; ++j)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        ROS_INFO("[objects] Parsed string has length < 1. (object colors for %s)", object_ids.back().c_str());
      if(!feof(fCfg) && strlen(sTemp) != 0)
        object_colors[i][j] = atof(sTemp);
    }
  }

  if(object_ids.size() != objects.size())
  {
    ROS_INFO("object id list is not same length as object list. exiting.");
    return false;
  }

  object.shapes.resize(1);
  object.poses.resize(1);
  object.shapes[0].dimensions.resize(3);
  if(transform_objects)
    ROS_WARN("[objects] Applying offset to the collision objects...");

  for(size_t i = 0; i < objects.size(); i++)
  {
    object.id = object_ids[i];
    object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
    object.shapes[0].type = arm_navigation_msgs::Shape::BOX;
    object.header.frame_id = frame_id;
    object.header.stamp = ros::Time::now();

    object.poses[0].position.x = objects[i][0];
    object.poses[0].position.y = objects[i][1];
    object.poses[0].position.z = objects[i][2];
    object.poses[0].orientation.x = 0;
    object.poses[0].orientation.y = 0;
    object.poses[0].orientation.z = 0;
    object.poses[0].orientation.w = 1;

    object.shapes[0].dimensions[0] = objects[i][3];
    object.shapes[0].dimensions[1] = objects[i][4];
    object.shapes[0].dimensions[2] = objects[i][5];

    // apply collision object offset (right now just translates and rotates about z)
    if(transform_objects)
    {
      if(!object_transform_v.empty())
      {
        geometry_msgs::Pose p, p2;
        Eigen::Affine3d a;
        a(0,0) = cos(object_transform_v[5]);
        a(1,0) = sin(object_transform_v[5]);
        a(0,1) = -sin(object_transform_v[5]);
        a(1,1) = cos(object_transform_v[5]);
        leatherman::msgFromPose(a, p2);
        leatherman::multiplyPoses(p2, object.poses[0], p);
        p.position.x += object_transform.position.x;
        p.position.y += object_transform.position.y;
        p.position.z += object_transform.position.z;
        object.poses[0] = p;
      }
      else
        ROS_ERROR("[objects] Expecting to translate/rotate collision objects but offset not found.");
    }

    collision_objects.push_back(object);
    ROS_INFO("[objects] [%d] id: %s xyz: %0.3f %0.3f %0.3f dims: %0.3f %0.3f %0.3f colors: %2.0f %2.0f %2.0f %2.0f",int(i),object_ids[i].c_str(),objects[i][0],objects[i][1],objects[i][2],objects[i][3],objects[i][4],objects[i][5], object_colors[i][0], object_colors[i][1], object_colors[i][2], object_colors[i][3]);

    std::vector<double> dim(3,0);
    dim[0] = objects[i][3];
    dim[1] = objects[i][4];
    dim[2] = objects[i][5];
    marker = viz::getCubeMarker(object.poses[0], dim, object_colors[i], frame_id, "collision_objects", int(i));
    object_markers.markers.push_back(marker);
  }
  return true;
}

void convertShapeToSolidPrimitive(const arm_navigation_msgs::Shape &shape, shape_msgs::SolidPrimitive &solid)
{
  if(shape.type == arm_navigation_msgs::Shape::SPHERE)
    solid.type = shape_msgs::SolidPrimitive::SPHERE;
  else if(shape.type == arm_navigation_msgs::Shape::BOX)
    solid.type = shape_msgs::SolidPrimitive::BOX;
  else if(shape.type == arm_navigation_msgs::Shape::CYLINDER)
    solid.type = shape_msgs::SolidPrimitive::CYLINDER;
  else if(shape.type == arm_navigation_msgs::Shape::MESH)
    ROS_ERROR("[objects] Cannot convert shape of type MESH to SolidPrimitive.");
  else
    ROS_ERROR("[objects] Invalid shape type '%d'.", shape.type);

  if(shape.type == arm_navigation_msgs::Shape::CYLINDER)
  {
    solid.dimensions.resize(2);
    solid.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = shape.dimensions[0];
    solid.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = shape.dimensions[1];
  }
  else
    solid.dimensions = shape.dimensions;
}

}
