#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/Point.h>

namespace leatherman
{

  Eigen::Vector3d trinorm(const Eigen::Vector3d &v0, const Eigen::Vector3d &v1, const Eigen::Vector3d &v2);

  bool readSTL(FILE *f, std::vector<int> &triangles, std::vector<Eigen::Vector3d> &vertices);

  bool readSTL(std::string filename, std::vector<int> &triangles, std::vector<Eigen::Vector3d> &vertices);

  bool writeSTL(std::string filename, const std::vector<int> &triangles, const std::vector<geometry_msgs::Point> &vertices, bool binary=true);

  bool writeSTL(FILE *f, const std::vector<int> &triangles, const std::vector<Eigen::Vector3d> &vertices);
  
  bool writeSTL(std::string filename, const std::vector<int> &triangles, const std::vector<Eigen::Vector3d> &vertices);

  /* ASCII STL file (inspired by http://people.sc.fsu.edu/~jburkardt/cpp_src/stla_io/stla_io.cpp) */
  bool writeSTLA(std::string filename, const std::vector<int> &triangles, const std::vector<Eigen::Vector3d> &vertices);
}

