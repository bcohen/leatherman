#include <ros/ros.h>
#include <sys/stat.h>
#include <dirent.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace leatherman
{
  bool createFolder(std::string name);

  bool writeJointTrajectoryToFile(FILE** file, const trajectory_msgs::JointTrajectory &traj);

}
