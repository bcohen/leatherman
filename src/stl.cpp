/* This code was copied (then modified) from the trimesh library
 * http://gfx.cs.princeton.edu/proj/trimesh2/
 * It was written by Szymon Rusinkiewicz at Princeton University
*/

#include <leatherman/stl.h>

#define FWRITE(ptr, size, nmemb, stream) do { if (fwrite((ptr), (size), (nmemb), (stream)) != (nmemb)) return false; } while (0)
#define COND_READ(cond, where, len) do { if ((cond) && !fread((void *)&(where), (len), 1, f)) return false; } while (0)


Eigen::Vector3d leatherman::trinorm(const Eigen::Vector3d &v0, const Eigen::Vector3d &v1, const Eigen::Vector3d &v2)
{
  return 0.5 * ((v1 - v0).cross(v2 - v0));
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

bool leatherman::writeSTL(FILE *f, const std::vector<int> &triangles, const std::vector<Eigen::Vector3d> &vertices)
{
  char header[80];
  memset(header, ' ', 80);
  FWRITE(header, 80, 1, f);

  int nfaces = triangles.size();
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
    vertices.push_back(Eigen::Vector3d(fbuf[3], fbuf[4], fbuf[5]));
    vertices.push_back(Eigen::Vector3d(fbuf[6], fbuf[7], fbuf[8]));
    vertices.push_back(Eigen::Vector3d(fbuf[9], fbuf[10], fbuf[11]));
    triangles.push_back(v);
    triangles.push_back(v+1);
    triangles.push_back(v+2);
    unsigned char att[2];
    COND_READ(true, att, 2);
  }
  return true;
}

