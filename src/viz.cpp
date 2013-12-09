/* \author Benjamin Cohen */

#include <leatherman/viz.h>
#include <tf/LinearMath/Quaternion.h>

visualization_msgs::Marker viz::getCubeMarker(geometry_msgs::PoseStamped pose, std::vector<double> dim, int hue, std::string ns, int id)
{
  std::vector<double> color(4,1);
  leatherman::HSVtoRGB(&(color[0]), &(color[1]), &(color[2]), hue, 1.0, 1.0);
  return getCubeMarker(pose.pose, dim, color, pose.header.frame_id, ns, id);
}

visualization_msgs::Marker viz::getCubeMarker(std::vector<double> &cube, std::vector<double> &color, std::string frame_id, std::string ns, int id)
{
  if(cube.size() < 6)
  {
    ROS_ERROR("Expecting a cube vector with 6 values. {x,y,z,dimx,dimy,dimz}");
    return visualization_msgs::Marker();
  }

  std::vector<double> dim(3,0);
  dim[0] = cube[3]; dim[1] = cube[4]; dim[2] = cube[5];
  
  geometry_msgs::Pose pose;
  pose.position.x = cube[0];
  pose.position.y= cube[1];
  pose.position.z = cube[2];
  pose.orientation.w = 1;
  return getCubeMarker(pose, dim, color, frame_id, ns, id);
}

visualization_msgs::Marker viz::getCubeMarker(geometry_msgs::Pose &pose, std::vector<double> &dim, std::vector<double> &color, std::string frame_id, std::string ns, int id)
{
  visualization_msgs::Marker marker;

  if(dim.size() < 3)
  {
    ROS_INFO("Three dimensions are needed to visualize a cube.");
    return marker;
  }
  if(color.size() < 4)
  {
    ROS_ERROR("No color specified.");
    return marker;
  }

  for(size_t i = 0; i < color.size(); ++i)
  {
    if(color[i] > 1)
      color[i] = color[i] / 255.0;
  }

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = pose;
  marker.scale.x = dim[0];
  marker.scale.y = dim[1];
  marker.scale.z = dim[2];
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];
  marker.lifetime = ros::Duration(0);
  return marker;
}

visualization_msgs::MarkerArray viz::getPosesMarkerArray(const std::vector<std::vector<double> > &poses, std::string frame_id, std::string ns, int id)
{
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.resize(poses.size()*3);
  tf::Quaternion pose_quaternion;
  geometry_msgs::Quaternion quaternion_msg;
  ros::Time time = ros::Time::now();

  for(int i = 0; i < (int)poses.size(); ++i)
  {
    pose_quaternion.setRPY(poses[i][3],poses[i][4],poses[i][5]);
    tf::quaternionTFToMsg(pose_quaternion, quaternion_msg);

    marker_array.markers[i*3].header.stamp = time;
    marker_array.markers[i*3].header.frame_id = frame_id;
    marker_array.markers[i*3].ns = ns;
    marker_array.markers[i*3].type = visualization_msgs::Marker::ARROW;
    marker_array.markers[i*3].id = id+(i*3);
    marker_array.markers[i*3].action = visualization_msgs::Marker::ADD;
    marker_array.markers[i*3].pose.position.x = poses[i][0];
    marker_array.markers[i*3].pose.position.y = poses[i][1];
    marker_array.markers[i*3].pose.position.z = poses[i][2];
    marker_array.markers[i*3].pose.orientation = quaternion_msg;
    marker_array.markers[i*3].scale.x = 0.1;
    marker_array.markers[i*3].scale.y = 0.015;
    marker_array.markers[i*3].scale.z = 0.015;
    marker_array.markers[i*3].color.r = 0.0;
    marker_array.markers[i*3].color.g = 0.7;
    marker_array.markers[i*3].color.b = 0.6;
    marker_array.markers[i*3].color.a = 0.7;
    marker_array.markers[i*3].lifetime = ros::Duration(0);
    marker_array.markers[i*3+1].header.stamp = time;
    marker_array.markers[i*3+1].header.frame_id = frame_id;
    marker_array.markers[i*3+1].ns = ns;
    marker_array.markers[i*3+1].id = id+(i*3)+1;
    marker_array.markers[i*3+1].type = visualization_msgs::Marker::SPHERE;
    marker_array.markers[i*3+1].action = visualization_msgs::Marker::ADD;
    marker_array.markers[i*3+1].pose.position.x = poses[i][0];
    marker_array.markers[i*3+1].pose.position.y = poses[i][1];
    marker_array.markers[i*3+1].pose.position.z = poses[i][2];
    marker_array.markers[i*3+1].pose.orientation = quaternion_msg;
    marker_array.markers[i*3+1].scale.x = 0.07;
    marker_array.markers[i*3+1].scale.y = 0.07;
    marker_array.markers[i*3+1].scale.z = 0.07;
    marker_array.markers[i*3+1].color.r = 1.0;
    marker_array.markers[i*3+1].color.g = 0.0;
    marker_array.markers[i*3+1].color.b = 0.6;
    marker_array.markers[i*3+1].color.a = 0.8;
    marker_array.markers[i*3+1].lifetime = ros::Duration(0);
    marker_array.markers[i*3+2].header.stamp = time;
    marker_array.markers[i*3+2].header.frame_id = frame_id;
    marker_array.markers[i*3+2].ns = ns;
    marker_array.markers[i*3+2].id = id+(i*3)+2;
    marker_array.markers[i*3+2].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_array.markers[i*3+2].action = visualization_msgs::Marker::ADD;
    marker_array.markers[i*3+2].pose.position.x = poses[i][0];
    marker_array.markers[i*3+2].pose.position.y = poses[i][1];
    marker_array.markers[i*3+2].pose.position.z = poses[i][2];
    marker_array.markers[i*3+2].scale.x = 0.015;
    marker_array.markers[i*3+2].scale.y = 0.015;
    marker_array.markers[i*3+2].scale.z = 0.015;
    marker_array.markers[i*3+2].color.r = 1.0;
    marker_array.markers[i*3+2].color.g = 1.0;
    marker_array.markers[i*3+2].color.b = 1.0;
    marker_array.markers[i*3+2].color.a = 0.95;
    marker_array.markers[i*3+2].text = boost::lexical_cast<std::string>(i+1);
    marker_array.markers[i*3+2].lifetime = ros::Duration(0);
  }
  return marker_array;
}

visualization_msgs::MarkerArray viz::getPoseMarkerArray(const geometry_msgs::Pose &pose, std::string frame_id, std::string ns, int id)
{
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.resize(3);
  ros::Time time = ros::Time::now();
  
  marker_array.markers[0].header.stamp = time;
  marker_array.markers[0].header.frame_id = frame_id;
  marker_array.markers[0].ns = ns;
  marker_array.markers[0].type = visualization_msgs::Marker::ARROW;
  marker_array.markers[0].id = 0+id;
  marker_array.markers[0].action = visualization_msgs::Marker::ADD;
  marker_array.markers[0].pose = pose;
  marker_array.markers[0].scale.x = 0.1;
  marker_array.markers[0].scale.y = 0.015;
  marker_array.markers[0].scale.z = 0.015;
  marker_array.markers[0].color.r = 0.0;
  marker_array.markers[0].color.g = 0.7;
  marker_array.markers[0].color.b = 0.6;
  marker_array.markers[0].color.a = 0.7;
  marker_array.markers[0].lifetime = ros::Duration(0);
  marker_array.markers[1].header.stamp = time;
  marker_array.markers[1].header.frame_id = frame_id;
  marker_array.markers[1].ns = ns;
  marker_array.markers[1].id = 1+id;
  marker_array.markers[1].type = visualization_msgs::Marker::SPHERE;
  marker_array.markers[1].action = visualization_msgs::Marker::ADD;
  marker_array.markers[1].pose = pose;
  marker_array.markers[1].scale.x = 0.07;
  marker_array.markers[1].scale.y = 0.07;
  marker_array.markers[1].scale.z = 0.10;
  marker_array.markers[1].color.r = 1.0;
  marker_array.markers[1].color.g = 0.0;
  marker_array.markers[1].color.b = 0.6;
  marker_array.markers[1].color.a = 0.6;
  marker_array.markers[1].lifetime = ros::Duration(0);
  marker_array.markers[2].header.stamp = time;
  marker_array.markers[2].header.frame_id = frame_id;
  marker_array.markers[2].ns = ns;
  marker_array.markers[2].id = 2+id;
  marker_array.markers[2].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker_array.markers[2].action = visualization_msgs::Marker::ADD;
  marker_array.markers[2].pose = pose;
  marker_array.markers[2].pose.position.z += 0.05;
  marker_array.markers[2].scale.x = 0.03;
  marker_array.markers[2].scale.y = 0.03;
  marker_array.markers[2].scale.z = 0.03;
  marker_array.markers[2].color.r = 1.0;
  marker_array.markers[2].color.g = 1.0;
  marker_array.markers[2].color.b = 1.0;
  marker_array.markers[2].color.a = 0.9;
  marker_array.markers[2].text = ns;
  marker_array.markers[2].lifetime = ros::Duration(0);
  return marker_array;
}

visualization_msgs::MarkerArray viz::getPoseMarkerArray(const geometry_msgs::PoseStamped &pose, std::string ns, int id)
{
  return viz::getPoseMarkerArray(pose.pose, pose.header.frame_id, ns, id);
}

visualization_msgs::Marker viz::getSphereMarker(double x, double y, double z, double radius, int hue, std::string frame_id, std::string ns, int id)
{
  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;
  leatherman::HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.scale.x = radius*2;
  marker.scale.y = radius*2;
  marker.scale.z = radius*2;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(0);
  return marker;
}

visualization_msgs::Marker viz::getSpheresMarker(const std::vector<std::vector<double> > &pose, double radius, int hue, std::string frame_id, std::string ns, int id)
{
  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;
  leatherman::HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = radius*2.0;
  marker.scale.y = radius*2.0;
  marker.scale.z = radius*2.0;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 0.6;
  marker.lifetime = ros::Duration(0);
  marker.id = id;

  marker.points.resize(pose.size());
  for(size_t i = 0; i < pose.size(); ++i)
  {
    marker.points[i].x = pose[i][0];
    marker.points[i].y = pose[i][1];
    marker.points[i].z = pose[i][2];
  }

  return marker;
}

visualization_msgs::Marker viz::getSpheresMarker(const std::vector<geometry_msgs::Point> &poses, double radius, int hue, std::string frame_id, std::string ns, int id)
{
  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;
  leatherman::HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

  if(poses.empty())
  {
    ROS_ERROR("The list of spheres is empty. Nothing to visualize.");
    return marker;
  }

  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time();
  marker.header.seq = 0;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = 0;
  marker.scale.x = radius;
  marker.scale.y = radius;
  marker.scale.z = radius;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 0.8;
  marker.lifetime = ros::Duration(0);
  marker.points = poses;
  return marker;
}

visualization_msgs::MarkerArray viz::getSpheresMarkerArray(const std::vector<std::vector<double> > &pose, std::vector<double> &radius, int hue, std::string frame_id, std::string ns, int id)
{
  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray marker_array;
  leatherman::HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 0.6;
  marker.lifetime = ros::Duration(0);

  for(size_t i = 0; i < pose.size(); ++i)
  {
    marker.id = id+i;
    marker.scale.x = radius[i]*2.0;
    marker.scale.y = radius[i]*2.0;
    marker.scale.z = radius[i]*2.0;
    marker.pose.position.x = pose[i][0];
    marker.pose.position.y = pose[i][1];
    marker.pose.position.z = pose[i][2];
    marker_array.markers.push_back(marker);
  }
  return marker_array;
}

visualization_msgs::MarkerArray viz::getSpheresMarkerArray(const std::vector<std::vector<double> > &pose, const std::vector<int> &hue, std::string frame_id, std::string ns, int id)
{
  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray marker_array;

  if(pose.size() != hue.size())
  {
    ROS_WARN("Didn't receive as many colors as I did spheres. Not visualizing. (spheres: %d, colors: %d)", int(pose.size()), int(hue.size()));
    return marker_array;
  }
  
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(0);
  marker.color.a = 0.6;

  for(std::size_t i = 0; i < pose.size(); ++i)
  {
    leatherman::HSVtoRGB(&r, &g, &b, hue[i], 1.0, 1.0);
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.id = id+i;
    marker.scale.x = pose[i][3]*2.0;
    marker.scale.y = pose[i][3]*2.0;
    marker.scale.z = pose[i][3]*2.0;
    marker.pose.position.x = pose[i][0];
    marker.pose.position.y = pose[i][1];
    marker.pose.position.z = pose[i][2];
    marker_array.markers.push_back(marker);
  }
  return marker_array;
}

visualization_msgs::MarkerArray viz::getRemoveMarkerArray(std::string ns, int max_id)
{
  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray marker_array; 
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.action = visualization_msgs::Marker::DELETE;
  marker_array.markers.resize(max_id,marker);
  for(int j = 0; j < max_id; ++j)
    marker_array.markers[j].id = j;
  return marker_array;
}

visualization_msgs::Marker viz::getCubesMarker(const std::vector<std::vector<double> > &poses, double size, const std::vector<double> &color, std::string frame_id, std::string ns, int id)
{
  visualization_msgs::Marker marker;
  
  //check if the list is empty
  if(poses.empty())
  {
    ROS_DEBUG("There are no poses in the %s poses list.", ns.c_str());
    return marker;
  }

  if(poses.size() > 50000)
    ROS_WARN("RViz may crash and burn when visualizing so many cubes.");

  marker.header.seq = 0;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];
  marker.lifetime = ros::Duration(0);

  marker.points.resize(poses.size());
  for(size_t i = 0; i < poses.size(); ++i)
  {
    if(poses[i].size() >= 3)
    {
      marker.points[i].x = poses[i][0];
      marker.points[i].y = poses[i][1];
      marker.points[i].z = poses[i][2];
    }
    else
      ROS_WARN("A cube pose was received with fewer than 3 values {x,y,z}. Not visualizing.");
  }

  return marker;
}

visualization_msgs::MarkerArray viz::getCubesMarkerArray(const std::vector<std::vector<double> > &poses, double size, const std::vector<std::vector<double> >&color, std::string frame_id, std::string ns, int id)
{
  std::vector<double> scaled_color(4,0);
  visualization_msgs::MarkerArray marker_array;

  //check if the list is empty
  if(poses.empty())
  {
    ROS_WARN("There are no poses in the %s poses list", ns.c_str());
    return marker_array;
  } 
    
  if(color.size()<2)
  {
    ROS_INFO("Not enough colors specified. Expecting two colors {2x4}.");
    return marker_array;
  } 
  
  if(color[0].size() < 4 || color[1].size() < 4)
  {
    ROS_INFO("RGBA must be specified for each color.");
    return marker_array;
  } 
 
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action =  visualization_msgs::Marker::ADD;
  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;
  marker.lifetime = ros::Duration(0);
  marker_array.markers.resize(poses.size(), marker);

  for(std::size_t i = 0; i < poses.size(); ++i)
  {
    for(unsigned int j = 0; j < 4; ++j)
      scaled_color[j] = color[0][j] - ((color[0][j] - color[1][j]) * (double(i)/double(poses.size())));
      
    marker_array.markers[i].id = id+i;
    marker_array.markers[i].color.r = scaled_color[0];
    marker_array.markers[i].color.g = scaled_color[1];
    marker_array.markers[i].color.b = scaled_color[2];
    marker_array.markers[i].color.a = scaled_color[3]; 
    marker_array.markers[i].pose.position.x = poses[i][0];
    marker_array.markers[i].pose.position.y = poses[i][1];
    marker_array.markers[i].pose.position.z = poses[i][2];
  } 
  return marker_array;
}

visualization_msgs::Marker viz::getLineMarker(const std::vector<geometry_msgs::Point> points, double thickness, int hue, std::string frame_id, std::string ns, int id)
{
  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;
  leatherman::HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.points = points;
  marker.scale.x = thickness;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 0.5;
  marker.lifetime = ros::Duration(0);
  return marker;
}

visualization_msgs::Marker viz::getTextMarker(geometry_msgs::Pose pose, std::string text, double size, int hue, std::string frame_id, std::string ns, int id)
{
  std::vector<double> color(4,1);
  leatherman::HSVtoRGB(&(color[0]), &(color[1]), &(color[2]), hue, 1.0, 1.0);

  return viz::getTextMarker(pose, text, size, color, frame_id, ns, id); 
}

visualization_msgs::Marker viz::getTextMarker(geometry_msgs::Pose pose, std::string text, double size, std::vector<double> color, std::string frame_id, std::string ns, int id)
{
  visualization_msgs::Marker marker;

  if(color.size() < 4)
    color.resize(4,1);

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;
  marker.pose = pose;
  
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];
  marker.text = text;
  marker.lifetime = ros::Duration(0);
  return marker;
}



visualization_msgs::Marker viz::getMeshMarker(const geometry_msgs::PoseStamped &pose, const std::string &mesh_resource, int hue, std::string ns, int id)
{
	double r = 0.0, g = 0.0, b = 0.0;
  if(hue > 0)
  	leatherman::HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

	visualization_msgs::Marker marker;
	marker.header.frame_id = pose.header.frame_id;
	marker.header.stamp = ros::Time::now();
	marker.ns = ns;
	marker.id = id;
	marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = pose.pose.position.x;
	marker.pose.position.y = pose.pose.position.y;
	marker.pose.position.z = pose.pose.position.z;
	marker.pose.orientation.x = pose.pose.orientation.x;
	marker.pose.orientation.y = pose.pose.orientation.y;
	marker.pose.orientation.z = pose.pose.orientation.z;
	marker.pose.orientation.w = pose.pose.orientation.w;
	marker.scale.x = 1.0;
	marker.scale.y = 1.0;
	marker.scale.z = 1.0;
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.color.a = 1.0;
	marker.mesh_resource = mesh_resource;

  if(hue < 0)
    marker.mesh_use_embedded_materials = true;

  return marker;
}

visualization_msgs::Marker viz::getMeshMarker(const geometry_msgs::PoseStamped &pose, const std::vector<geometry_msgs::Point> &vertices, const std::vector<int> &triangles, int hue, bool psychadelic, std::string ns, int id)
{
	double r = 0.0, g = 0.0, b = 0.0;
	leatherman::HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

	std_msgs::ColorRGBA red; red.a = 1.0f; red.r = 1.0f; red.g = 0.0f; red.b = 0.0f;
	std_msgs::ColorRGBA green; green.a = 1.0f; green.r = 0.0f; green.g = 1.0f; green.b = 0.0f;
	std_msgs::ColorRGBA blue; blue.a = 1.0f; blue.r = 0.0f; blue.g = 0.0f; blue.b = 1.0f;

	std::vector<std_msgs::ColorRGBA> colors;
	for (int i = 0; i < (int)vertices.size(); i++)
  {
		if (i % 3 == 0) colors.push_back(red);
		if (i % 3 == 1) colors.push_back(green);
		if (i % 3 == 2) colors.push_back(blue);
	}

	visualization_msgs::Marker marker;
	marker.header.frame_id = pose.header.frame_id;
	marker.header.stamp = ros::Time::now();
	marker.ns = ns;
	marker.id = id;
	marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = pose.pose.position.x;
	marker.pose.position.y = pose.pose.position.y;
	marker.pose.position.z = pose.pose.position.z;
	marker.pose.orientation.x = pose.pose.orientation.x;
	marker.pose.orientation.y = pose.pose.orientation.y;
	marker.pose.orientation.z = pose.pose.orientation.z;
	marker.pose.orientation.w = pose.pose.orientation.w;
	marker.scale.x = 1.0;
	marker.scale.y = 1.0;
	marker.scale.z = 1.0;
	marker.points = vertices;

	if (psychadelic)
  {
		marker.color.r = 1.0;
		marker.color.g = 1.0;
		marker.color.b = 1.0;
		marker.color.a = 1.0;
		marker.colors = colors;
	}
	else
  {
		marker.color.r = r;
		marker.color.g = g;
		marker.color.b = b;
		marker.color.a = 1.0;
	}
	return marker;
}

visualization_msgs::MarkerArray viz::getShapesMarkerArray(const std::vector<arm_navigation_msgs::Shape> &shapes, const std::vector<geometry_msgs::Pose> &poses, const std::vector<std::vector<double> >&color, std::string frame_id, std::string ns, int id)
{
  visualization_msgs::Marker m;
  visualization_msgs::MarkerArray ma;

  if(poses.size() != shapes.size())
  {
    ROS_WARN("The shapes and poses arrays must be of the same length. (shapes: %d  poses: %d)", int(shapes.size()), int(poses.size()));
    return ma;
  } 
    
  m.header.frame_id = frame_id;
  m.header.stamp = ros::Time::now();
  m.ns = ns;
  m.action =  visualization_msgs::Marker::ADD;
  m.lifetime = ros::Duration(0);

  for(std::size_t i = 0; i < shapes.size(); ++i)
  { 
    m.id = id+i;
    m.color.r = color[i][0];
    m.color.g = color[i][1];
    m.color.b = color[i][2];
    m.color.a = color[i][3]; 
    m.pose = poses[i];
  
    if(shapes[i].type == arm_navigation_msgs::Shape::BOX)
    { 
      m.type = visualization_msgs::Marker::CUBE;
      m.scale.x = shapes[i].dimensions[0];
      m.scale.y = shapes[i].dimensions[1];
      m.scale.z = shapes[i].dimensions[2];
      ma.markers.push_back(m);
    }
    else if(shapes[i].type == arm_navigation_msgs::Shape::SPHERE)
    { 
      m.type = visualization_msgs::Marker::SPHERE;
      m.scale.x = shapes[i].dimensions[0];
      m.scale.y = shapes[i].dimensions[0];
      m.scale.z = shapes[i].dimensions[0];
      ma.markers.push_back(m);
    }
    else if(shapes[i].type == arm_navigation_msgs::Shape::CYLINDER)
    { 
      m.type = visualization_msgs::Marker::CYLINDER;
      m.scale.x = shapes[i].dimensions[0];
      m.scale.y = shapes[i].dimensions[0];
      m.scale.z = shapes[i].dimensions[1];
      ma.markers.push_back(m);
    }
    else
      ROS_ERROR("Shapes of type '%d' are not supported yet.", int(shapes[i].type));
  }
  return ma;
}

visualization_msgs::MarkerArray viz::getCollisionObjectMarkerArray(const arm_navigation_msgs::CollisionObject &obj, const std::vector<double> &hue, std::string ns, int id)
{
  std::vector<std::vector<double> > color(hue.size(), std::vector<double>(4,1.0));
  for(size_t i = 0; i < color.size(); ++i)
    leatherman::HSVtoRGB(&(color[i][0]), &(color[i][1]), &(color[i][2]), hue[i], 1.0, 1.0);

  return getShapesMarkerArray(obj.shapes, obj.poses, color, obj.header.frame_id, ns, id);  
}

