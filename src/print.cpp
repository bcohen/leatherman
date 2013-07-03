#include <leatherman/print.h>

void leatherman::printPoseMsg(const geometry_msgs::Pose &p, std::string text)
{
  ROS_INFO("[%s] xyz: %0.3f %0.3f %0.3f  quat: %0.3f %0.3f %0.3f %0.3f", text.c_str(), p.position.x, p.position.y, p.position.z, p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
}

void leatherman::printPoseStampedMsg(const geometry_msgs::PoseStamped &p, std::string text)
{
  ROS_INFO("[%s] xyz: %0.3f %0.3f %0.3f  quat: %0.3f %0.3f %0.3f %0.3f  time: % 0.3f", text.c_str(), p.pose.position.x, p.pose.position.y, p.pose.position.z, p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w, p.header.stamp.toSec());
}

void leatherman::printPose(const std::vector<double> &p, std::string text)
{
  ROS_INFO("[%s] xyz: % 0.3f % 0.3f % 0.3f  rpy: % 0.3f % 0.3f % 0.3f", text.c_str(), p[0], p[1], p[2], p[3], p[4], p[5]);
}

void leatherman::printJointTrajectory(const trajectory_msgs::JointTrajectory &traj, std::string text)
{
  if(!traj.points.empty())
    ROS_INFO("[%s] start_time: %0.3f", text.c_str(), traj.header.stamp.toSec());
  for(unsigned int i = 0; i < traj.points.size(); i++)
  {
    ROS_INFO("[%s] [% 2d] % 1.4f % 1.4f % 1.4f % 1.4f % 1.4f % 1.4f % 1.4f   time_from_start: % 0.3f", text.c_str(), i,traj.points[i].positions[0],traj.points[i].positions[1],traj.points[i].positions[2],traj.points[i].positions[3],traj.points[i].positions[4],traj.points[i].positions[5],traj.points[i].positions[6], traj.points[i].time_from_start.toSec());
  }
}

void leatherman::printJointTrajectoryPoints(const std::vector<trajectory_msgs::JointTrajectoryPoint> &points, std::string text)
{
  for(unsigned int i = 0; i < points.size(); i++)
  {
    ROS_INFO("[%s] [% 2d] % 1.4f % 1.4f % 1.4f % 1.4f % 1.4f % 1.4f % 1.4f   time_from_start: % 0.3f", text.c_str(), i,points[i].positions[0],points[i].positions[1],points[i].positions[2],points[i].positions[3],points[i].positions[4],points[i].positions[5],points[i].positions[6], points[i].time_from_start.toSec());
  }
}
void leatherman::printCompleteJointTrajectory(const trajectory_msgs::JointTrajectory &traj, std::string name)
{
  for(size_t i = 0; i < traj.points.size(); ++i)
  {
    printf("[%d] time_from_start: %1.4f\n", int(i), traj.points[i].time_from_start.toSec());
    // positions
    if(traj.points[i].positions.size() > 0)
    {
      printf("       positions: ");
      for(size_t j = 0; j < traj.points[i].positions.size(); ++j)
        printf("%1.2f, ", traj.points[i].positions[j]);
      printf("\n");
    }
    // velocities
    if(traj.points[i].velocities.size() > 0)
    {
      printf("      velocities: ");
      for(size_t j = 0; j < traj.points[i].velocities.size(); ++j)
        printf("%1.2f, ", traj.points[i].velocities[j]);
      printf("\n");
    }
    // accelerations
    if(traj.points[i].accelerations.size() > 0)
    {
      printf("   accelerations: ");
      for(size_t j = 0; j < traj.points[i].accelerations.size(); ++j)
        printf("%1.2f, ", traj.points[i].accelerations[j]);
      printf("\n");
    }
  }
}

void leatherman::printAffine3d(const Eigen::Affine3d &a, std::string text)
{
  ROS_INFO("[%s] xyz: %0.3f %0.3f %0.3f", text.c_str(), a.translation().x(), a.translation().y(), a.translation().z());
}

void leatherman::printKDLFrame(const KDL::Frame &f, std::string text)
{
  double r,p,y; f.M.GetRPY(r,p,y);
  ROS_INFO("[%s] xyz: %0.3f %0.3f %0.3f  rpy: %0.3f %0.3f %0.3f", text.c_str(), f.p[0], f.p[1], f.p[2], r, p, y);
}

void leatherman::printKDLFrames(const std::vector<std::vector<KDL::Frame> > &f, std::string text)
{
  double r,p,y;
  for(size_t i = 0; i < f.size(); ++i)
  {
    for(size_t j = 0; j < f[i].size(); ++j)
    {
      f[i][j].M.GetRPY(r,p,y);
      ROS_INFO("[%s] [%d %d]  xyz: %0.3f %0.3f %0.3f  rpy: %0.3f %0.3f %0.3f", text.c_str(), int(i), int(j), f[i][j].p.x(), f[i][j].p.y(), f[i][j].p.z(), r, p, y);
    }
  }
}


void leatherman::printKDLChain(const KDL::Chain &c, std::string text)
{
  ROS_INFO("[%s] # segments: %d  # joints: %d", text.c_str(), c.getNrOfSegments(), c.getNrOfJoints());
  double r,p,y,jnt_pos = 0;
  for(size_t j = 0; j < c.getNrOfSegments(); ++j)
  {
    c.getSegment(j).pose(jnt_pos).M.GetRPY(r,p,y);
    ROS_INFO("[%s] [%2d] segment: %21s  xyz: %0.3f %0.3f %0.3f  rpy: %0.3f %0.3f %0.3f", 
        text.c_str(), int(j),
        c.getSegment(j).getName().c_str(),
        c.getSegment(j).pose(jnt_pos).p.x(),
        c.getSegment(j).pose(jnt_pos).p.y(),
        c.getSegment(j).pose(jnt_pos).p.z(),
        r,p,y);

    c.getSegment(j).getJoint().pose(jnt_pos).M.GetRPY(r,p,y);
    ROS_INFO("[%s]        joint: %21s  xyz: %0.3f %0.3f %0.3f  rpy: %0.3f %0.3f %0.3f  type: %s", 
        text.c_str(),
        c.getSegment(j).getJoint().getName().c_str(),
        c.getSegment(j).getJoint().pose(jnt_pos).p.x(),
        c.getSegment(j).getJoint().pose(jnt_pos).p.y(),
        c.getSegment(j).getJoint().pose(jnt_pos).p.z(),
        r,p,y,
        c.getSegment(j).getJoint().getTypeName().c_str());

  }
}

std::string leatherman::getString(const std::vector<double> &v, int precision)
{
  std::stringstream ss;
  for(std::size_t j = 0; j < v.size(); ++j)
    ss << std::fixed << std::setw(precision) << std::setprecision(precision) << std::showpoint << v[j] << " ";
  return ss.str();
}

std::string leatherman::getString(const std::vector<bool> &v, std::string t, std::string f)
{
  std::stringstream ss;
  for(std::size_t j = 0; j < v.size(); ++j)
  {
    if(v[j])
      ss << t << " ";
    else
      ss << f << " ";
  }
  return ss.str();
}
