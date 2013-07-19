#include <vector>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace leatherman
{

  Eigen::Vector3d trinorm(const Eigen::Vector3d &v0, const Eigen::Vector3d &v1, const Eigen::Vector3d &v2);

  bool readSTL(FILE *f, std::vector<int> &triangles, std::vector<Eigen::Vector3d> &vertices);

  bool readSTL(std::string filename, std::vector<int> &triangles, std::vector<Eigen::Vector3d> &vertices);

  bool writeSTL(FILE *f, const std::vector<int> &triangles, const std::vector<Eigen::Vector3d> &vertices);
  
  bool writeSTL(std::string filename, const std::vector<int> &triangles, const std::vector<Eigen::Vector3d> &vertices);
}
