#ifndef _LEATHERMAN_PRINT_
#define _LEATHERMAN_PRINT_

#include <ros/console.h>
#include <vector>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <kdl/frames.hpp>
#include <kdl/chain.hpp>

namespace leatherman
{
  void printPose(const std::vector<double> &p, std::string text);
  void printPoseMsg(const geometry_msgs::Pose &p, std::string text);
  void printPoseStampedMsg(const geometry_msgs::PoseStamped &p, std::string text);
  void printJointTrajectory(const trajectory_msgs::JointTrajectory &traj, std::string text);
  void printJointTrajectoryPoints(const std::vector<trajectory_msgs::JointTrajectoryPoint> &points, std::string text);
  void printCompleteJointTrajectory(const trajectory_msgs::JointTrajectory &traj, std::string name);

  void printAffine3d(const Eigen::Affine3d &a, std::string text);
  void printKDLFrame(const KDL::Frame &f, std::string text);
  void printKDLFrames(const std::vector<std::vector<KDL::Frame> > &f, std::string text);
  void printKDLChain(const KDL::Chain &c, std::string text);

  std::string getString(const std::vector<int> &v);
  std::string getString(const std::vector<double> &v, int precision=3);
  std::string getString(const std::vector<bool> &v, std::string t, std::string f);
}

#endif
