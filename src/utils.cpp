#include <leatherman/utils.h>

#define SMALL_NUM  0.00000001     // to avoid division overflow

double leatherman::distanceBetween3DLineSegments(std::vector<int> l1a, std::vector<int> l1b,std::vector<int> l2a, std::vector<int> l2b)
{
  // Copyright 2001, softSurfer (www.softsurfer.com)
  // This code may be freely used and modified for any purpose
  // providing that this copyright notice is included with it.
  // SoftSurfer makes no warranty for this code, and cannot be held
  // liable for any real or imagined damage resulting from its use.
  // Users of this code must verify correctness for their application.

  double u[3];
  double v[3];
  double w[3];
  double dP[3];

  u[0] = l1b[0] - l1a[0];
  u[1] = l1b[1] - l1a[1];
  u[2] = l1b[2] - l1a[2];

  v[0] = l2b[0] - l2a[0];
  v[1] = l2b[1] - l2a[1];
  v[2] = l2b[2] - l2a[2];

  w[0] = l1a[0] - l2a[0];
  w[1] = l1a[1] - l2a[1];
  w[2] = l1a[2] - l2a[2];

  double a = u[0] * u[0] + u[1] * u[1] + u[2] * u[2]; // always >= 0
  double b = u[0] * v[0] + u[1] * v[1] + u[2] * v[2]; // dot(u,v);
  double c = v[0] * v[0] + v[1] * v[1] + v[2] * v[2]; // dot(v,v);        // always >= 0
  double d = u[0] * w[0] + u[1] * w[1] + u[2] * w[2]; // dot(u,w);
  double e = v[0] * w[0] + v[1] * w[1] + v[2] * w[2]; // dot(v,w);
  double D = a*c - b*b;       // always >= 0
  double sc, sN, sD = D;      // sc = sN / sD, default sD = D >= 0
  double tc, tN, tD = D;      // tc = tN / tD, default tD = D >= 0

  // compute the line parameters of the two closest points
  if (D < SMALL_NUM) { // the lines are almost parallel
    sN = 0.0;        // force using point P0 on segment S1
    sD = 1.0;        // to prevent possible division by 0.0 later
    tN = e;
    tD = c;
  }
  else {                // get the closest points on the infinite lines
    sN = (b*e - c*d);
    tN = (a*e - b*d);
    if (sN < 0.0) {       // sc < 0 => the s=0 edge is visible
      sN = 0.0;
      tN = e;
      tD = c;
    }
    else if (sN > sD) {  // sc > 1 => the s=1 edge is visible
      sN = sD;
      tN = e + b;
      tD = c;
    }
  }

  if (tN < 0.0) {           // tc < 0 => the t=0 edge is visible
    tN = 0.0;
    // recompute sc for this edge
    if (-d < 0.0)
      sN = 0.0;
    else if (-d > a)
      sN = sD;
    else {
      sN = -d;
      sD = a;
    }
  }
  else if (tN > tD) {      // tc > 1 => the t=1 edge is visible
    tN = tD;
    // recompute sc for this edge
    if ((-d + b) < 0.0)
      sN = 0;
    else if ((-d + b) > a)
      sN = sD;
    else {
      sN = (-d + b);
      sD = a;
    }
  }

  // finally do the division to get sc and tc
  sc = (fabs(sN) < SMALL_NUM ? 0.0 : sN / sD);
  tc = (fabs(tN) < SMALL_NUM ? 0.0 : tN / tD);

  // get the difference of the two closest points
  // dP = w + (sc * u) - (tc * v);  // = S1(sc) - S2(tc)

  dP[0] = w[0] + (sc * u[0]) - (tc * v[0]);
  dP[1] = w[1] + (sc * u[1]) - (tc * v[1]);
  dP[2] = w[2] + (sc * u[2]) - (tc * v[2]);

  return  sqrt(dP[0]*dP[0] + dP[1]*dP[1] + dP[2]*dP[2]);   // return the closest distance
}

/* Originally from geometric_shapes/shape_operations. Then it 
 * was moved to pr2_navigation/load_mesh.cpp
 * Written by Ioan Sucan */

shapes::Mesh* leatherman::createMeshFromBinaryStlData(const char *data, unsigned int size)
{
  const char* pos = data;
  pos += 80; // skip the 80 byte header

  unsigned int numTriangles = *(unsigned int*)pos;
  pos += 4;

  // make sure we have read enough data
  if ((long)(50 * numTriangles + 84) <= size)
  {
    std::vector<tf::Vector3> vertices;

    for (unsigned int currentTriangle = 0 ; currentTriangle < numTriangles ; ++currentTriangle)
    {
      // skip the normal
      pos += 12;

      // read vertices 
      btVector3 v1(0,0,0);
      btVector3 v2(0,0,0);
      btVector3 v3(0,0,0);

      v1.setX(*(float*)pos);
      pos += 4;
      v1.setY(*(float*)pos);
      pos += 4;
      v1.setZ(*(float*)pos);
      pos += 4;

      v2.setX(*(float*)pos);
      pos += 4;
      v2.setY(*(float*)pos);
      pos += 4;
      v2.setZ(*(float*)pos);
      pos += 4;

      v3.setX(*(float*)pos);
      pos += 4;
      v3.setY(*(float*)pos);
      pos += 4;
      v3.setZ(*(float*)pos);
      pos += 4;

      // skip attribute
      pos += 2;

      vertices.push_back(v1);
      vertices.push_back(v2);
      vertices.push_back(v3);
    }

    return shapes::createMeshFromVertices(vertices);
  }

  return NULL;
}

shapes::Mesh* leatherman::createMeshFromBinaryStl(const char *filename)
{
  FILE* input = fopen(filename, "r");
  if (!input)
    return NULL;

  fseek(input, 0, SEEK_END);
  long fileSize = ftell(input);
  fseek(input, 0, SEEK_SET);

  char* buffer = new char[fileSize];
  size_t rd = fread(buffer, fileSize, 1, input);

  fclose(input);

  shapes::Mesh *result = NULL;

  if (rd == 1)
    result = createMeshFromBinaryStlData(buffer, fileSize);

  delete[] buffer;

  return result;
}

void leatherman::getMeshComponents(shapes::Mesh* mesh, std::vector<int> &triangles, std::vector<geometry_msgs::Point> &vertices)
{
  geometry_msgs::Point v;

  // copy vertices
  printf("vertexCount: %d    triangleCount: %d\n", mesh->vertexCount, mesh->triangleCount); fflush(stdout);
  for (unsigned int i = 0 ; i < mesh->vertexCount; ++i)
  {
    v.x = mesh->vertices[3 * i    ];
    v.y = mesh->vertices[3 * i + 1];
    v.z = mesh->vertices[3 * i + 2];
    vertices.push_back(v);
  }

  // copy triangles
  triangles.resize(mesh->triangleCount*3);
  for (unsigned int i = 0 ; i < mesh->triangleCount; ++i)
  {
    triangles[3 * i    ] = mesh->triangles[3 * i    ];
    triangles[3 * i + 1] = mesh->triangles[3 * i + 1];
    triangles[3 * i + 2] = mesh->triangles[3 * i + 2];
  }
}

void leatherman::poseVectorToPoseMsg(const std::vector<double> &pose, geometry_msgs::Pose &pose_msg)
{
  if(pose.size() <6)
    ROS_ERROR("Pose must have 6 elements. Cannot convert pose to pose message.");
  else
  {
    btQuaternion btpose;
    pose_msg.position.x = pose[0];
    pose_msg.position.y = pose[1];
    pose_msg.position.z = pose[2];
    btpose = setRPY(pose[3], pose[4], pose[5]);
    tf::quaternionTFToMsg(btpose, pose_msg.orientation);
  }
}

void leatherman::rpyToQuatMsg(double r, double p, double y, geometry_msgs::Quaternion &q)
{
  btQuaternion btpose;
  btpose = setRPY(r, p, y);
  tf::quaternionTFToMsg(btpose, q);
}

void leatherman::quatMsgToRPY(const geometry_msgs::Quaternion &q, double &r, double &p, double &y)
{
  tf::Transform tf_pose;
  geometry_msgs::Pose pose;
  pose.orientation = q;
  tf::poseMsgToTF(pose, tf_pose);
  tf_pose.getBasis().getRPY(r,p,y);
}

void leatherman::getRPY(const geometry_msgs::Quaternion &qmsg, double &roll, double &pitch, double &yaw)
{
  geometry_msgs::Pose pose;
  pose.orientation = qmsg;
  tf::Pose tf_pose;
  tf::poseMsgToTF(pose, tf_pose);
  tf_pose.getBasis().getRPY(roll,pitch,yaw);
  ROS_DEBUG("[utils] rpy: %0.3f %0.3f %0.3f  quat: %0.3f %0.3f %0.3f %0.3f\n", roll, pitch, yaw, qmsg.x, qmsg.y, qmsg.z, qmsg.w);fflush(stdout);
}

double leatherman::getYaw(const geometry_msgs::Quaternion &q)
{
  tf::Transform tf_pose;
  double r=0,p=0,y=0;
  geometry_msgs::Pose pose;
  pose.orientation = q;
  tf::poseMsgToTF(pose, tf_pose);
  tf_pose.getBasis().getRPY(r,p,y);
  return y;
}

void leatherman::poseMsgToPoseVector(const geometry_msgs::Pose &pose, std::vector<double> &posev)
{
  tf::Transform tf_pose;
  posev.resize(6);
  posev[0] = pose.position.x;
  posev[1] = pose.position.y;
  posev[2] = pose.position.z;
  tf::poseMsgToTF(pose, tf_pose);
  tf_pose.getBasis().getRPY(posev[3], posev[4], posev[5]);
}

void leatherman::multiplyPoses(geometry_msgs::Pose &p1, geometry_msgs::Pose &p2, geometry_msgs::Pose &p)
{
  Eigen::Affine3d p_e, p1_e, p2_e;
  poseFromMsg(p1, p1_e);
  poseFromMsg(p2, p2_e);
  p_e = p1_e * p2_e;
  msgFromPose(p_e, p);
}

void leatherman::multiply(const geometry_msgs::Pose &a, const geometry_msgs::Pose &b, geometry_msgs::Pose &c)
{
  tf::Transform bta, btb, btc;
  tf::poseMsgToTF(a, bta);
  tf::poseMsgToTF(b, btb);
  btc = bta * btb;
  tf::poseTFToMsg(btc, c);
}

bool leatherman::poseFromMsg(const geometry_msgs::Pose &tmsg, Eigen::Affine3d &t)
{
  Eigen::Quaterniond q; bool r = quatFromMsg(tmsg.orientation, q);
  t = Eigen::Affine3d(Eigen::Translation3d(tmsg.position.x, tmsg.position.y, tmsg.position.z)*q.toRotationMatrix());
  return r;
}

void leatherman::msgFromPose(const Eigen::Affine3d &t, geometry_msgs::Pose &tmsg)
{
  tmsg.position.x = t.translation().x(); tmsg.position.y = t.translation().y(); tmsg.position.z = t.translation().z();
  Eigen::Quaterniond q(t.rotation());
  tmsg.orientation.x = q.x(); tmsg.orientation.y = q.y(); tmsg.orientation.z = q.z(); tmsg.orientation.w = q.w();
}

void leatherman::msgFromPose(const Eigen::Affine3d &t, geometry_msgs::Transform &tmsg)
{
  tmsg.translation.x = t.translation().x(); tmsg.translation.y = t.translation().y(); tmsg.translation.z = t.translation().z();
  Eigen::Quaterniond q(t.rotation());
  tmsg.rotation.x = q.x(); tmsg.rotation.y = q.y(); tmsg.rotation.z = q.z(); tmsg.rotation.w = q.w();
}

bool leatherman::quatFromMsg(const geometry_msgs::Quaternion &qmsg, Eigen::Quaterniond &q)
{
  q = Eigen::Quaterniond(qmsg.w, qmsg.x, qmsg.y, qmsg.z);
  if (fabs(q.squaredNorm() - 1.0) > 1e-3)
  {
    q = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    return false;
  }
  return true;
}

void leatherman::comparePoses(const std::vector<double> &a, const std::vector<double> &b, std::vector<double> &diff)
{
  // computes absolute distance between poses
  diff.resize(6,0);
  if(a.size() < 6 || b.size() < 6)
    return;

  diff[0] = a[0] - b[0];
  diff[1] = a[1] - b[1];
  diff[2] = a[2] - b[2];
  diff[3] = angles::shortest_angular_distance(a[3], b[3]);
  diff[4] = angles::shortest_angular_distance(a[4], b[4]);
  diff[5] = angles::shortest_angular_distance(a[5], b[5]);
}

void leatherman::comparePosesAbsolute(const std::vector<double> &a, const std::vector<double> &b, std::vector<double> &diff)
{
  comparePoses(a,b,diff);

  for(size_t i = 0; i < diff.size(); ++i)
    diff[i] = fabs(diff[i]);
}

bool leatherman::getIntermediatePoints(trajectory_msgs::JointTrajectoryPoint a, trajectory_msgs::JointTrajectoryPoint b, int num_points, std::vector<trajectory_msgs::JointTrajectoryPoint> &points)
{
  if(a.positions.size() != b.positions.size())
    return false;
    
  double time_inc = (b.time_from_start - a.time_from_start).toSec() / (num_points+1);
  std::vector<double> inc(a.positions.size(),0);
  for(size_t i = 0; i < a.positions.size(); ++i)
    inc[i] = angles::shortest_angular_distance(a.positions[i], b.positions[i]) / (num_points+1);
     
  
  points.resize(num_points);
  for(int i = 0; i < num_points; ++i)
  {
    points[i].positions.resize(a.positions.size());
    for(size_t j = 0; j < a.positions.size(); ++j)
      points[i].positions[j] = a.positions[j] + (i+1)*inc[j];
    points[i].time_from_start = a.time_from_start + ros::Duration((i+1)*(time_inc));
  }
  return true; 
} 

btQuaternion leatherman::setRPY(const btScalar& roll, const btScalar& pitch, const btScalar& yaw)
{
  btScalar halfYaw = btScalar(yaw) * btScalar(0.5);
  btScalar halfPitch = btScalar(pitch) * btScalar(0.5);
  btScalar halfRoll = btScalar(roll) * btScalar(0.5);
  btScalar cosYaw = btCos(halfYaw);
  btScalar sinYaw = btSin(halfYaw);
  btScalar cosPitch = btCos(halfPitch);
  btScalar sinPitch = btSin(halfPitch);
  btScalar cosRoll = btCos(halfRoll);
  btScalar sinRoll = btSin(halfRoll);
  btQuaternion q;
  q.setValue(sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw, //x
      cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw, //y
      cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw, //z
      cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw); //formerly yzx

  return q;
}

void leatherman::setRPY(double roll, double pitch, double yaw, Eigen::Matrix3d &m)
{
  m = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
}

void leatherman::getRPY(const Eigen::Matrix3d &m, double &roll, double &pitch, double &yaw)
{
  Eigen::Vector3d v = m.eulerAngles(0,1,2);
  roll = v(0);  pitch = v(1);  yaw = v(2);
}

void leatherman::transformKDLToEigen(const KDL::Frame &k, Eigen::Affine3d &e)
{
  e(0,3) = k.p[0];
  e(1,3) = k.p[1];
  e(2,3) = k.p[2];

  e(0,0) = k.M(0,0);
  e(0,1) = k.M(0,1);
  e(0,2) = k.M(0,2);
  e(1,0) = k.M(1,0);
  e(1,1) = k.M(1,1);
  e(1,2) = k.M(1,2);
  e(2,0) = k.M(2,0);
  e(2,1) = k.M(2,1);
  e(2,2) = k.M(2,2);

  e(3,0) = 0.0;
  e(3,1) = 0.0;
  e(3,2) = 0.0;
  e(3,3) = 1.0;
}

void leatherman::transformEigenToKDL(const Eigen::Affine3d &e, KDL::Frame &k)
{
  k.p[0] = e(0,3);
  k.p[1] = e(1,3);
  k.p[2] = e(2,3);

  k.M(0,0) = e(0,0);
  k.M(0,1) = e(0,1);
  k.M(0,2) = e(0,2);
  k.M(1,0) = e(1,0);
  k.M(1,1) = e(1,1);
  k.M(1,2) = e(1,2);
  k.M(2,0) = e(2,0);
  k.M(2,1) = e(2,1);
  k.M(2,2) = e(2,2);
}

double leatherman::distance(const KDL::Vector &a, const KDL::Vector &b)
{
  return sqrt((a.x()-b.x())*(a.x()-b.x()) + (a.y()-b.y())*(a.y()-b.y()) + (a.z()-b.z())*(a.z()-b.z()));
}

double leatherman::distance(const Eigen::Vector3d &a, const Eigen::Vector3d &b)
{
  return sqrt((a(0)-b(0))*(a(0)-b(0)) + (a(1)-b(1))*(a(1)-b(1)) + (a(2)-b(2))*(a(2)-b(2)));
}

void leatherman::getIntermediatePoints(Eigen::Vector3d a, Eigen::Vector3d b, double d, std::vector<Eigen::Vector3d> &points)
{
  Eigen::Vector3d pt, dir;
  int interm_points = floor(leatherman::distance(a,b) / d + 0.5);

  dir = b - a;
  dir.normalize();
  ROS_DEBUG("# interm points: %d  unit vector: %0.3f %0.3f %0.3f", interm_points, dir(0), dir(1), dir(2));

  points.clear();
  points.push_back(a);
  for(int i = 1; i <= interm_points; ++i)
  {
    pt = a + dir*i*d;
    points.push_back(pt);
  }
  points.push_back(b);
}


void leatherman::getIntermediatePoints(KDL::Vector a, KDL::Vector b, double d, std::vector<KDL::Vector> &points)
{
  KDL::Vector pt, dir;
  int interm_points = floor(leatherman::distance(a,b) / d + 0.5);

  dir = b - a;
  double norm  = dir.Normalize();
  ROS_DEBUG("# interm points: %d  unit vector: %0.3f %0.3f %0.3f norm: %0.3f", interm_points, dir.x(), dir.y(), dir.z(), norm);

  points.clear();
  points.push_back(a);
  for(int i = 1; i <= interm_points; ++i)
  {
    pt = a + dir*i*d;
    points.push_back(pt);
  }
  points.push_back(b);
}

bool leatherman::findJointPosition(const sensor_msgs::JointState &state, std::string name, double &position)
{
  for(size_t i = 0; i < state.name.size(); i++)
  {
    if(name.compare(state.name[i]) == 0)
    {
      position = state.position[i];
      return true;
    }
  }
  return false;
}

bool leatherman::getJointPositions(const sensor_msgs::JointState &state, std::vector<std::string> &names, std::vector<double> &positions)
{
  size_t nind = 0;
  positions.resize(names.size());
  for(size_t i = 0; i < state.position.size(); ++i)
  {
    if(names[nind].compare(state.name[i]) == 0)
    {
      positions[nind] = state.position[i];
      nind++;
    } 
    if(nind == names.size())
      break;
  }
  if(nind != names.size())
    return false;

  return true; 
}

bool leatherman::getJointIndex(const KDL::Chain &c, std::string name, int &index)
{
  for(size_t j = 0; j < c.getNrOfSegments(); ++j)
  {
    if(c.getSegment(j).getJoint().getName().compare(name) == 0)
    {
      index = j;
      return true;
    }
  }
  return false;
}

