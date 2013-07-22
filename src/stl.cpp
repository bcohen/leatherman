/* This code was copied (then modified) from the trimesh library
 * http://gfx.cs.princeton.edu/proj/trimesh2/
 * It was written by Szymon Rusinkiewicz at Princeton University
*/

#include <leatherman/stl.h>

#define FWRITE(ptr, size, nmemb, stream) do { if (fwrite((ptr), (size), (nmemb), (stream)) != (nmemb)) return false; } while (0)
#define COND_READ(cond, where, len) do { if ((cond) && !fread((void *)&(where), (len), 1, f)) return false; } while (0)


Eigen::Vector3d leatherman::trinorm(const Eigen::Vector3d &v0, const Eigen::Vector3d &v1, const Eigen::Vector3d &v2)
{
  Eigen::Vector3d a = v1 - v0;
  Eigen::Vector3d b = v2 - v0;
  Eigen::Vector3d c = a.cross(b);
  return c;
  //return ((v1 - v0).cross(v2 - v0));
}

bool leatherman::readSTL(std::string filename, std::vector<int> &triangles, std::vector<Eigen::Vector3d> &vertices)
{
  FILE* f = fopen(filename.c_str(), "r");
  if(!f)
    return false;

  return leatherman::readSTL(f, triangles, vertices);
}

bool leatherman::writeSTL(std::string filename, const std::vector<int> &triangles, const std::vector<Eigen::Vector3d> &vertices)
{
  FILE* f = fopen(filename.c_str(), "w");
  if(!f)
    return false;

  return leatherman::writeSTL(f, triangles, vertices);
}

bool leatherman::writeSTL(std::string filename, const std::vector<int> &triangles, const std::vector<geometry_msgs::Point> &vertices, bool binary)
{
  std::vector<Eigen::Vector3d> v(vertices.size());
  for(size_t p = 0; p < vertices.size(); ++p)
  {
    v[p](0) = vertices[p].x;
    v[p](1) = vertices[p].y;
    v[p](2) = vertices[p].z;
  }

  if(binary)
    return leatherman::writeSTL(filename, triangles, v);
  else  
    return leatherman::writeSTLA(filename, triangles, v);
}

bool leatherman::writeSTL(FILE *f, const std::vector<int> &triangles, const std::vector<Eigen::Vector3d> &vertices)
{
  char header[80];
  memset(header, ' ', 80);
  FWRITE(header, 80, 1, f);

  int nfaces = triangles.size() / 3;
  FWRITE(&nfaces, 4, 1, f);

  for (size_t i = 0; i < triangles.size(); i=i+3)
  {
    float fbuf[12];
    Eigen::Vector3d tn = trinorm(vertices[triangles[i]], vertices[triangles[i+1]], vertices[triangles[i+2]]);
    tn = tn.normalized();
    fbuf[0] = tn[0]; fbuf[1] = tn[1]; fbuf[2] = tn[2];
    fbuf[3]  = vertices[triangles[i]].x();
    fbuf[4]  = vertices[triangles[i]].y();
    fbuf[5]  = vertices[triangles[i]].z();
    fbuf[6]  = vertices[triangles[i+1]].x();
    fbuf[7]  = vertices[triangles[i+1]].y();
    fbuf[8]  = vertices[triangles[i+1]].z();
    fbuf[9]  = vertices[triangles[i+2]].x();
    fbuf[10] = vertices[triangles[i+2]].y();
    fbuf[11] = vertices[triangles[i+2]].z();
    FWRITE(fbuf, 48, 1, f);
    unsigned char att[2] = { 0, 0 };
    FWRITE(att, 2, 1, f);
  }
  return true;
}

bool leatherman::readSTL(FILE *f, std::vector<int> &triangles, std::vector<Eigen::Vector3d> &vertices)
{
  char header[80];
  COND_READ(true, header, 80);

  int nfacets;
  COND_READ(true, nfacets, 4);
  
  triangles.reserve(3*nfacets);
  vertices.reserve(3*nfacets);
  for (int i = 0; i < nfacets; i++)
  {
    float fbuf[12];
    COND_READ(true, fbuf, 48);
    int v = vertices.size();
    ROS_INFO("vertex  %0.3f  %0.3f  %0.3f",fbuf[3], fbuf[4], fbuf[5]);
    vertices.push_back(Eigen::Vector3d(fbuf[3], fbuf[4], fbuf[5]));
    ROS_INFO("vertex  %0.3f  %0.3f  %0.3f",fbuf[6], fbuf[7], fbuf[8]);
    vertices.push_back(Eigen::Vector3d(fbuf[6], fbuf[7], fbuf[8]));
    ROS_INFO("vertex  %0.3f  %0.3f  %0.3f",fbuf[9], fbuf[10], fbuf[11]);
    vertices.push_back(Eigen::Vector3d(fbuf[9], fbuf[10], fbuf[11]));
    triangles.push_back(v);
    triangles.push_back(v+1);
    triangles.push_back(v+2);
    unsigned char att[2];
    COND_READ(true, att, 2);
  }
  return true;
}

bool leatherman::writeSTLA(std::string filename, const std::vector<int> &triangles, const std::vector<Eigen::Vector3d> &vertices)
{
  std::ofstream output_unit;
  output_unit.open(filename.c_str());

  if(!output_unit)
  {
    ROS_ERROR("Failed to open the file for writing.");
    return false;
  }

  output_unit << "solid MYSOLID\n";
  for (size_t face = 0; face < triangles.size(); face+=3)
  {
    Eigen::Vector3d tn = leatherman::trinorm(vertices[triangles[face]], vertices[triangles[face+1]], vertices[triangles[face+2]]);
    tn = tn.normalized();
    output_unit << "  facet normal";
    output_unit << "  " << std::setw(10) << tn.x();
    output_unit << "  " << std::setw(10) << tn.y();
    output_unit << "  " << std::setw(10) << tn.z();
    output_unit << "\n";
    output_unit << "    outer loop\n";
    output_unit << "      vertex  "; 
    output_unit << "  " << std::setw(10) << vertices[triangles[face]].x();
    output_unit << "  " << std::setw(10) << vertices[triangles[face]].y();
    output_unit << "  " << std::setw(10) << vertices[triangles[face]].z();
    output_unit << "\n";
    output_unit << "      vertex  ";
    output_unit << "  " << std::setw(10) << vertices[triangles[face+1]].x();
    output_unit << "  " << std::setw(10) << vertices[triangles[face+1]].y();
    output_unit << "  " << std::setw(10) << vertices[triangles[face+1]].z();
    output_unit << "\n";
    output_unit << "      vertex  ";
    output_unit << "  " << std::setw(10) << vertices[triangles[face+2]].x();
    output_unit << "  " << std::setw(10) << vertices[triangles[face+2]].y();
    output_unit << "  " << std::setw(10) << vertices[triangles[face+2]].z();
    output_unit << "\n";
    output_unit << "    end loop\n";
    output_unit << "  end facet\n";
  }
  output_unit << "end solid MYSOLID\n";
  output_unit.close();
  return false;
}

