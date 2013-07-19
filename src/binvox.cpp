#include <leatherman/binvox.h>
#include <leatherman/file.h>
#include <cstring>

leatherman::VoxelGrid& Grid()
{
  static leatherman::VoxelGrid grid;
  return grid;
}

bool leatherman::ReadBinvox(std::string filename)
{
  ROS_INFO("Reading binvox file: %s", filename.c_str());
  std::ifstream* input = new std::ifstream(filename.c_str(), std::ios::in | std::ios::binary);

  // read header
  std::string line;
  *input >> line;  // #binvox
  if (line.compare("#binvox") != 0) {
    ROS_INFO_STREAM("Error: first line reads [" << line << "] instead of [#binvox]");
    delete input;
    return false;
  }
  *input >> Grid().version;
  ROS_INFO_STREAM("reading binvox version " << Grid().version);

  Grid().depth = -1;
  int done = 0;
  while (input->good() && !done) {
    *input >> line;
    if (line.compare("data") == 0) {
      done = 1;
    }
    else if (line.compare("dim") == 0) {
      *input >> Grid().depth >> Grid().height >> Grid().width;
    } else if (line.compare("translate") == 0) {
      *input >> Grid().tx >> Grid().ty >> Grid().tz;
    } else if (line.compare("scale") == 0) {
      *input >> Grid().scale;
    } else {
      ROS_INFO_STREAM("  unrecognized keyword [" << line << "], skipping");
      char c;
      do {
        c = input->get(); // skip until end of line
      }
      while (input->good() && (c != '\n'));
    }
  }

  if (!done) {
    ROS_INFO_STREAM("  error reading header");
    delete input;
    return false;
  }
  if (Grid().depth == -1) {
    ROS_INFO_STREAM("  missing dimensions in header");
    delete input;
    return false;
  }

  Grid().size = Grid().width * Grid().height * Grid().depth;
  Grid().voxels = new byte[Grid().size];
  if (!Grid().voxels) {
    ROS_INFO_STREAM("  error allocating memory");
    delete input;
    return false;
  }

  //
  // read voxel data
  //
  byte value;
  byte count;
  int index = 0;
  int end_index = 0;
  int nr_voxels = 0;

  input->unsetf(std::ios::skipws);  // need to read every byte now (!)
  *input >> value;  // read the linefeed char

  while ((end_index < Grid().size) && input->good()) {
    *input >> value >> count;

    if (input->good()) {
      end_index = index + count;
      if (end_index > Grid().size) {
        delete input;
        return false;
      }
      for (int i = index; i < end_index; i++)
        Grid().voxels[i] = value;

      if (value)
        nr_voxels += count;
      index = end_index;
    }  // if file still ok
  }  // while

  input->close();
  delete input;
  ROS_INFO_STREAM("  read " << nr_voxels << " voxels");

  return true;
}

bool leatherman::InsertMesh(std::string filename, const boost::shared_ptr<octomap::Pointcloud>& point_cloud)
{
  if (!ReadBinvox(filename)) {
    ROS_ERROR_STREAM("Error reading [" << filename << "]");
    return false;
  }

  for (int x = 0; x < Grid().width; x++) {
    //        for (int y = 0; y < Grid().height; y++) {
    for (int y = Grid().height - 1; y >= 0; y--) {
      for (int z = 0; z < Grid().depth; z++) {
        int index = z * (Grid().height * Grid().width) + y * Grid().width + x;
        if (Grid().voxels[index]) {
          float xf = 0.0254f * Grid().scale * ((float)x / Grid().width); //+ Grid().tx;
          float zf = 0.0254f * Grid().scale * ((float)y / Grid().height); //+ Grid().ty;
          float yf = 0.0254f * Grid().scale * ((float)z / Grid().depth); //+ Grid().tz;
          //                    point_cloud->push_back(yf, xf, zf);
          point_cloud->push_back(xf, -yf, zf);
        }
      }
    }
  }
  return true;
}

bool leatherman::convertBinvoxToOctomapMsg(std::string filename, double resolution, octomap_msgs::Octomap &msg)
{
  boost::shared_ptr<octomap::OcTree> octree(new octomap::OcTree(resolution));
  boost::shared_ptr<octomap::Pointcloud> point_cloud(new octomap::Pointcloud());

  InsertMesh(filename, point_cloud);

  ROS_INFO("Inserting scan with %lu points.", point_cloud->size());
  octree->insertScan(*point_cloud, octomap::point3d(0.0f, 0.0f, 0.0f));

  ROS_INFO("Converting to octomap_msgs/Octomap.");
  octomap_msgs::binaryMapToMsg(*octree, msg);
  return false;
}

bool leatherman::convertBinvoxToPointCloudMsg(std::string filename, sensor_msgs::PointCloud &pc)
{
  boost::shared_ptr<octomap::Pointcloud> point_cloud(new octomap::Pointcloud());
  if(!leatherman::InsertMesh(filename, point_cloud))
    return false;

  pc.header.seq = 0;
  pc.header.stamp = ros::Time::now();
  pc.header.frame_id = "/map";
  pc.channels.clear();
  for(octomap::point3d_collection::iterator it = point_cloud->begin(); it != point_cloud->end(); ++it)
  {
    geometry_msgs::Point32 p;
    p.x = it->x();
    p.y = it->y();
    p.z = it->z();
    pc.points.push_back(p);
  }
  return true;
}

bool leatherman::convertBinvoxToVector3d(std::string filename, std::vector<Eigen::Vector3d> &voxels)
{
  sensor_msgs::PointCloud pc;
  if(!leatherman::convertBinvoxToPointCloudMsg(filename, pc))
    return false;

  voxels.resize(pc.points.size());
  for(size_t i = 0; i < pc.points.size(); ++i)
  {
    voxels[i](0) = pc.points[i].x; 
    voxels[i](1) = pc.points[i].y; 
    voxels[i](2) = pc.points[i].z; 
  }

  return true;
}

bool leatherman::createBinvoxFile(std::string mesh_filename, std::string &binvox_filename)
{
  int result = 1000;
  std::string command = "binvox -e  " + mesh_filename;
  try{
    result = system(command.c_str());
  }
  catch(int e){
    ROS_ERROR("Failed to call binvox. {exception: %d  result: %d}", e, result);
    return false;
  }
  ROS_INFO("system call result: %d", result);

  // this is error prone if run twice in same folder on same file
  std::string path = leatherman::getPathWithoutFilename(mesh_filename);
  std::string filename = leatherman::getFilenameFromPath(mesh_filename, true);
  binvox_filename = path + filename + ".binvox";
  return true;
}

bool leatherman::convertBinvoxToBtSystem(std::string binvox_filename, std::string &bt_filename)
{
  int result = 1000;
  std::string path = leatherman::getPathWithoutFilename(binvox_filename);
  std::string filename = leatherman::getFilenameFromPath(binvox_filename, true);
  bt_filename = path + filename + ".bt";

  std::string command = "binvox2bt --mark-free -o " + bt_filename + " " + binvox_filename;
  try{
    result = system(command.c_str());
  }
  catch(int e){
    ROS_ERROR("Failed to call binvox2bt. {exception: %d  result: %d}", e, result);
    return false;
  }
  ROS_DEBUG("system call result: %d", result);
  return true;
}

bool leatherman::voxelizeMesh(std::string filename, double resolution, std::vector<Eigen::Vector3d> &voxels)
{
  ros::Time t_start = ros::Time::now();
  std::string binvox_filename;

  // binvox it
  leatherman::createBinvoxFile(filename, binvox_filename);

  // parse binvox file - Andrew's function
  // return leatherman::convertBinvoxToVector3d(binvox_filename, voxels);

  // convert to list of points
  if(!getOccupiedVoxelsInBinvoxFile(binvox_filename, voxels))
    return false;

  ROS_WARN("Voxelizing the mesh took %0.3fsec", ros::Duration(ros::Time::now()-t_start).toSec());
  return true;
}

void leatherman::convertOcTreeToCollisionMap(octomap::OcTree &octree, arm_navigation_msgs::CollisionMap &cmap)
{
  cmap.header.frame_id = "/map";
  cmap.header.stamp = ros::Time::now();
  arm_navigation_msgs::OrientedBoundingBox collObjBox;
  collObjBox.axis.x = collObjBox.axis.y = 0.0;
  collObjBox.axis.z = 1.0;
  collObjBox.angle = 0.0;

  for (octomap::OcTree::iterator it = octree.begin(octree.getTreeDepth()), end = octree.end(); it != end; ++it)
  {
    if (octree.isNodeOccupied(*it))
    {
      collObjBox.extents.x = it.getSize();
      collObjBox.extents.y = it.getSize();
      collObjBox.extents.z = it.getSize();
      collObjBox.center.x = it.getX();
      collObjBox.center.y = it.getY();
      collObjBox.center.z = it.getZ();
      cmap.boxes.push_back(collObjBox);
    }
  }
}

void leatherman::getOccupiedVoxelsInOcTree(octomap::OcTree* octree, std::vector<Eigen::Vector3d> &voxels)
{
  Eigen::Vector3d v;
  for (octomap::OcTree::iterator it = octree->begin(octree->getTreeDepth()), end = octree->end(); it != end; ++it)
  {
    if (octree->isNodeOccupied(*it))
    {
      v(0) = it.getX();
      v(1) = it.getY();
      v(2) = it.getZ();
      voxels.push_back(v);
    }
  }
}

void leatherman::getOccupiedVoxelsInCollisionMap(const arm_navigation_msgs::CollisionMap &map, std::vector<Eigen::Vector3d> &voxels)
{
  voxels.resize(map.boxes.size());
  for(size_t i = 0; i < map.boxes.size(); ++i)
  {
    voxels[i](0) = map.boxes[i].center.x;
    voxels[i](1) = map.boxes[i].center.y;
    voxels[i](2) = map.boxes[i].center.z;
  }
}


bool leatherman::getOccupiedVoxelsInBinvoxFile(std::string binvox_filename, std::vector<Eigen::Vector3d> &voxels)
{
  int version;               // binvox file format version (should be 1)
  int depth, height, width;  // dimensions of the voxel grid
  int size;                  // number of grid cells (height * width * depth)
  float tx, ty, tz;          // Translation
  float scale;               // Scaling factor
  octomap::point3d offset(0.0, 0.0, 0.0);
  octomap::OcTree* tree = 0;

  // Open input file
  std::ifstream *input = new std::ifstream(binvox_filename.c_str(), std::ios::in | std::ios::binary);    
  if(!input->good())
  {
    ROS_ERROR("Failed to open binvox file, '%s'.", binvox_filename.c_str());
    return false;
  }
  
  // read header
  std::string line;
  *input >> line;    // #binvox
  if (line.compare("#binvox") != 0) {
    std::cout << "Error: first line reads [" << line << "] instead of [#binvox]" << std::endl;
    delete input;
    return 0;
  }
  *input >> version;
  std::cout << "reading binvox version " << version << std::endl;

  depth = -1;
  int done = 0;
  while(input->good() && !done) {
    *input >> line;
    if (line.compare("data") == 0) done = 1;
    else if (line.compare("dim") == 0) {
      *input >> depth >> height >> width;
    }
    else if (line.compare("translate") == 0) {
      *input >> tx >> ty >> tz;
    }
    else if (line.compare("scale") == 0) {
      *input >> scale;
    }
    else {
      std::cout << "    unrecognized keyword [" << line << "], skipping" << std::endl;
      char c;
      do {    // skip until end of line
        c = input->get();
      } while(input->good() && (c != '\n'));

    }
  }
  if (!done) {
    std::cout << "    error reading header" << std::endl;
    return 0;
  }
  if (depth == -1) {
    std::cout << "    missing dimensions in header" << std::endl;
    return 0;
  }

  size = width * height * depth;
  int maxSide = std::max(std::max(width, height), depth);
  double res = double(scale)/double(maxSide);
  ROS_ERROR("size: %d  width: %d  depth: %d  height: %d  maxSide: %d   res:  %f", size, width, depth, height, maxSide, res);
  if(!tree)
  {
    std::cout << "Generating octree with leaf size " << res << std::endl << std::endl;
    tree = new octomap::OcTree(res);
  }

  std::cout << "Read data: ";
  std::cout.flush();

  // read voxel data
  byte value;
  byte count;
  int index = 0;
  int end_index = 0;
  unsigned nr_voxels = 0;

  input->unsetf(std::ios::skipws);    // need to read every byte now (!)
  *input >> value;    // read the linefeed char

  while((end_index < size) && input->good()) 
  {
    *input >> value >> count;

    if (input->good())
    {
      end_index = index + count;
      if (end_index > size) return 0;
      for(int i=index; i < end_index; i++)
      {
        // Output progress dots
        if(i % (size / 20) == 0) {
          std::cout << ".";            
          std::cout.flush();
        }
        // voxel index --> voxel coordinates 
        int y = i % width;
        int z = (i / width) % height;
        int x = i / (width * height);

        // voxel coordinates --> world coordinates
        octomap::point3d endpoint(
            (float) ((double) x*res + tx + 0.000001), 
            (float) ((double) y*res + ty + 0.000001),
            (float) ((double) z*res + tz + 0.000001));

        // marks a node free if not occupied
        tree->updateNode(endpoint, value == 1, true);
      }

      if (value) nr_voxels += count;
      index = end_index;
    }    // if file still ok
  }    // while
  input->close();

  // prune octree
  std::cout << "Pruning octree" << std::endl << std::endl;
  tree->updateInnerOccupancy();
  tree->prune();
  ROS_INFO("Successfully created octree from binvox file with %d voxels.", nr_voxels);  

  Eigen::Vector3d v;
  for (octomap::OcTree::iterator it = tree->begin(tree->getTreeDepth()), end = tree->end(); it != end; ++it)
  {
    if (tree->isNodeOccupied(*it))
    {
      v(0) = it.getX();
      v(1) = it.getY();
      v(2) = it.getZ();
      voxels.push_back(v);
    }
  }
  return true;
}

bool leatherman::convertBinvoxToBt(std::string binvox_filename, std::string &bt_filename)
{
  int version;               // binvox file format version (should be 1)
  int depth, height, width;  // dimensions of the voxel grid
  int size;                  // number of grid cells (height * width * depth)
  float tx, ty, tz;          // Translation
  float scale;               // Scaling factor
  octomap::point3d offset(0.0, 0.0, 0.0);
  octomap::OcTree *tree = 0;

  // Open input file
  std::string output_filename;
  std::ifstream *input = new std::ifstream(binvox_filename.c_str(), std::ios::in | std::ios::binary);    
  if(!input->good()) {
    std::cerr << "Error: Could not open input file " << binvox_filename << "!" << std::endl;
    exit(1);
  }
  else {
    std::cout << "Reading binvox file " << binvox_filename << "." << std::endl;
    if(output_filename.empty())
    { 
      output_filename = std::string(binvox_filename).append(".bt");
    }
  }
  bt_filename = output_filename;

  // read header
  std::string line;
  *input >> line;    // #binvox
  if (line.compare("#binvox") != 0) {
    std::cout << "Error: first line reads [" << line << "] instead of [#binvox]" << std::endl;
    delete input;
    return 0;
  }
  *input >> version;
  std::cout << "reading binvox version " << version << std::endl;

  depth = -1;
  int done = 0;
  while(input->good() && !done) {
    *input >> line;
    if (line.compare("data") == 0) done = 1;
    else if (line.compare("dim") == 0) {
      *input >> depth >> height >> width;
    }
    else if (line.compare("translate") == 0) {
      *input >> tx >> ty >> tz;
    }
    else if (line.compare("scale") == 0) {
      *input >> scale;
    }
    else {
      std::cout << "    unrecognized keyword [" << line << "], skipping" << std::endl;
      char c;
      do {    // skip until end of line
        c = input->get();
      } while(input->good() && (c != '\n'));

    }
  }
  if (!done) {
    std::cout << "    error reading header" << std::endl;
    return 0;
  }
  if (depth == -1) {
    std::cout << "    missing dimensions in header" << std::endl;
    return 0;
  }

  size = width * height * depth;
  int maxSide = std::max(std::max(width, height), depth);
  double res = double(scale)/double(maxSide);

  if(!tree) {
    std::cout << "Generating octree with leaf size " << res << std::endl << std::endl;
    tree = new octomap::OcTree(res);
  }

  std::cout << "Read data: ";
  std::cout.flush();

  // read voxel data
  byte value;
  byte count;
  int index = 0;
  int end_index = 0;
  unsigned nr_voxels = 0;
  unsigned nr_voxels_out = 0;

  input->unsetf(std::ios::skipws);    // need to read every byte now (!)
  *input >> value;    // read the linefeed char

  while((end_index < size) && input->good()) 
  {
    *input >> value >> count;

    if (input->good()) {
      end_index = index + count;
      if (end_index > size) return 0;
      for(int i=index; i < end_index; i++) {
        // Output progress dots
        if(i % (size / 20) == 0) {
          std::cout << ".";            
          std::cout.flush();
        }
        // voxel index --> voxel coordinates 
        int y = i % width;
        int z = (i / width) % height;
        int x = i / (width * height);

        // voxel coordinates --> world coordinates
        octomap::point3d endpoint((float) ((double) x*res + tx + 0.000001), 
            (float) ((double) y*res + ty + 0.000001),
            (float) ((double) z*res + tz + 0.000001));

        // marks a node free if not occupied
        tree->updateNode(endpoint, value == 1, true);
      }

      if (value) nr_voxels += count;
      index = end_index;
    }    // if file still ok
  }    // while

  input->close();
  std::cout << "    read " << nr_voxels << " voxels, skipped "<<nr_voxels_out << " (out of bounding box)\n\n";
  
  // prune octree
  std::cout << "Pruning octree" << std::endl << std::endl;
  tree->updateInnerOccupancy();
  tree->prune();
  
  tree->writeBinary(output_filename.c_str());

  ROS_INFO("Done writing octree to bt file.");
  return true;
}

