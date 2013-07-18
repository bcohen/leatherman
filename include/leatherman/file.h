#ifndef _LEATHERMAN_FILE_
#define _LEATHERMAN_FILE_

#include <ros/console.h>
#include <sys/stat.h>
#include <dirent.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace leatherman
{
  bool createFolder(std::string name);

  bool writeJointTrajectoryToFile(FILE** file, const trajectory_msgs::JointTrajectory &traj);

  bool writePointsToFile(std::string filename, const std::vector<Eigen::Vector3d> &pts);

  bool readPointsInFile(std::string filename, std::vector<Eigen::Vector3d> &pts);
}

#endif
