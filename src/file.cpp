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

