#ifndef _LEATHERMAN_UTILS_
#define _LEATHERMAN_UTILS_

#include <ros/console.h>
#include <iostream>
#include <cstdlib>
#include <vector>
#include <string>
#include <geometric_shapes/shape_operations.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <LinearMath/btVector3.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <angles/angles.h>
#include <kdl/frames.hpp>
#include <kdl/chain.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>

namespace leatherman
{
  /* Conversions */
  void poseVectorToPoseMsg(const std::vector<double> &pose, geometry_msgs::Pose &pose_msg);
  void poseMsgToPoseVector(const geometry_msgs::Pose &pose, std::vector<double> &posev);
  bool quatFromMsg(const geometry_msgs::Quaternion &qmsg, Eigen::Quaterniond &q);
  bool poseFromMsg(const geometry_msgs::Pose &tmsg, Eigen::Affine3d &t);
  void msgFromPose(const Eigen::Affine3d &t, geometry_msgs::Pose &tmsg);
  void msgFromPose(const Eigen::Affine3d &t, geometry_msgs::Transform &tmsg);
  void quatMsgToRPY(const geometry_msgs::Quaternion &q, double &r, double &p, double &y);
  void rpyToQuatMsg(double r, double p, double y, geometry_msgs::Quaternion &q);
  void transformKDLToEigen(const KDL::Frame &k, Eigen::Affine3d &e);
  void transformEigenToKDL(const Eigen::Affine3d &e, KDL::Frame &k);

  double getYaw(const geometry_msgs::Quaternion &q);
  void setRPY(double roll, double pitch, double yaw, Eigen::Matrix3d &m);
  void getRPY(const Eigen::Matrix3d &m, double &roll, double &pitch, double &yaw);
  void getRPY(const geometry_msgs::Quaternion &qmsg, double &roll, double &pitch, double &yaw);
  btQuaternion setRPY(const btScalar& roll, const btScalar& pitch, const btScalar& yaw);

  /* Geometry */
  double distance(const KDL::Vector &a, const KDL::Vector &b);
  double distance(const Eigen::Vector3d &a, const Eigen::Vector3d &b);
  double distanceBetween3DLineSegments(std::vector<int> l1a, std::vector<int> l1b,std::vector<int> l2a,std::vector<int> l2b);
  void getIntermediatePoints(Eigen::Vector3d a, Eigen::Vector3d b, double d, std::vector<Eigen::Vector3d> &points);
  void getIntermediatePoints(KDL::Vector a, KDL::Vector b, double d, std::vector<KDL::Vector> &points);
  bool getIntermediatePoints(trajectory_msgs::JointTrajectoryPoint a, trajectory_msgs::JointTrajectoryPoint b, int num_points, std::vector<trajectory_msgs::JointTrajectoryPoint> &points);

  /* Meshes */
  shapes::Mesh* createMeshFromBinaryStl(const char *filename);
  shapes::Mesh* createMeshFromBinaryStlData(const char *data, unsigned int size);
  void getMeshComponents(shapes::Mesh* mesh, std::vector<int> &triangles, std::vector<geometry_msgs::Point> &vertices);

  /* Math */
  void multiplyPoses(geometry_msgs::Pose &p1, geometry_msgs::Pose &p2, geometry_msgs::Pose &p);
  void multiply(const geometry_msgs::Pose &a, const geometry_msgs::Pose &b, geometry_msgs::Pose &c);
  void comparePoses(const std::vector<double> &a, const std::vector<double> &b, std::vector<double> &diff);
  void comparePosesAbsolute(const std::vector<double> &a, const std::vector<double> &b, std::vector<double> &diff);

  /* Search (string comparisons) */
  bool findJointPosition(const sensor_msgs::JointState &state, std::string name, double &position);
  bool getJointPositions(const sensor_msgs::JointState &state, std::vector<std::string> &names, std::vector<double> &positions);
  bool getJointIndex(const KDL::Chain &c, std::string name, int &index);
}

#endif
