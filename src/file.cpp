#include <leatherman/file.h>

bool leatherman::createFolder(std::string name)
{
  struct stat st;
  if(mkdir(name.c_str(),  S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == 0)
    ROS_INFO("Successfully created the trajectory folder: %s", name.c_str());
  else
  {
    if(stat(name.c_str(), &st) == 0)
      ROS_INFO("Folder is present. Not creating.");
    else
    {
      ROS_WARN("Failed to create the trajectory folder: %s. Maybe it exists already?", name.c_str());
      return false;
    }
  }
  return true;
}

bool leatherman::writeJointTrajectoryToFile(FILE** file, const trajectory_msgs::JointTrajectory &traj)
{
  if(*file == NULL)
  {
    ROS_ERROR("File pointer is null. Not writing.");
    return false;
  }
  
  for(size_t i = 0; i < traj.points.size(); ++i)
  { 
    fprintf(*file, "%d, time_from_start, %1.4f, ", int(i), traj.points[i].time_from_start.toSec());
    // positions
    if(traj.points[i].positions.size() > 0)
    {
      fprintf(*file, "positions, ");
      for(size_t j = 0; j < traj.points[i].positions.size(); ++j)
        fprintf(*file, "%1.4f, ", traj.points[i].positions[j]);
    }
    // velocities
    if(traj.points[i].velocities.size() > 0)
    { 
      fprintf(*file, "velocities, ");
      for(size_t j = 0; j < traj.points[i].velocities.size(); ++j)
        fprintf(*file, "%1.4f, ", traj.points[i].velocities[j]);
    }
    // accelerations
    if(traj.points[i].accelerations.size() > 0)
    { 
      fprintf(*file, "accelerations, ");
      for(size_t j = 0; j < traj.points[i].accelerations.size(); ++j)
        fprintf(*file, "%1.4f, ", traj.points[i].accelerations[j]);
    }
    fprintf(*file, "\n");
  }   
      
  fflush(*file);
  return true;
}

bool leatherman::writePointsToFile(std::string filename, const std::vector<Eigen::Vector3d> &pts)
{
  FILE* f = NULL;
  if((f=fopen(filename.c_str(),"a")) == NULL)
  {
    ROS_ERROR("Failed to open file for writing. {%s}", filename.c_str());
    return false;
  }
  
  for(size_t i = 0; i < pts.size(); ++i)
    fprintf(f, "%1.4f, %1.4f, %1.4f\n", pts[i].x(), pts[i].y(), pts[i].z());
   
  fflush(f);
  return true;
}

bool leatherman::readPointsInFile(std::string filename, std::vector<Eigen::Vector3d> &pts)
{
  FILE* f = NULL;
  if((f=fopen(filename.c_str(),"r")) == NULL)
  {
    ROS_ERROR("Failed to open file for writing. {%s}", filename.c_str());
    return false;
  }
 
  Eigen::Vector3d v;
  float x,y,z;
  while(!feof(f))
  {
    if(fscanf(f,"%f, %f, %f\n", &x, &y, &z) < 1)
    {
      ROS_ERROR("Error while parsing list of points.");
      return false;
    }
    pts.push_back(Eigen::Vector3d(x, y, z));
  }

  fclose(f);
  return true;
}

std::string leatherman::getFilenameFromPath(std::string path, bool remove_extension)
{
  std::string filename;
  size_t pos = path.find_last_of("/");
  if(pos != std::string::npos)
    filename.assign(path.begin() + pos + 1, path.end());
  else
    filename = path;

  if(remove_extension)
  {
    pos = filename.find_last_of(".");
    if(pos != std::string::npos)
      filename.assign(filename.begin(),  filename.begin() + pos);
  }
  return filename;
}

std::string leatherman::getPathWithoutFilename(std::string path)
{
  // keeps final "/"

  std::string filename;
  size_t pos = path.find_last_of("/");
  if(pos != std::string::npos)
    filename.assign(path.begin(), path.begin()+pos+1);
  else
    filename = path;

  return filename;
}

bool leatherman::getSystemPathFromROSPath(std::string ros_path, std::string &system_path)
{
  std::string rp = ros_path;
  std::string pn, apn;
  
  // remove 'package://'
  size_t pos = rp.find_first_of("package://");
  if(pos != std::string::npos)
    rp.erase(pos, 10);
  else
  {
    ROS_ERROR("Not a ROS package path. (Failed to find 'package://')");
    return false;
  } 
  
  // get package name
  pos = rp.find_first_of("/");
  if(pos != std::string::npos)
  { 
    pn.assign(rp.begin(), rp.begin() + pos);
    apn.assign(rp.begin() + pos + 1, rp.end());
  }
  else
  { 
    ROS_ERROR("No slash found when searching for package name.");
    return false;
  }
  
  // get package path
  std::string package_path = ros::package::getPath(pn);
  if(package_path.empty())
  { 
    ROS_ERROR("Failed to get system path for package '%s'", pn.c_str());
    return false;
  }
  
  system_path = package_path + "/" + apn;
  return true;
}

