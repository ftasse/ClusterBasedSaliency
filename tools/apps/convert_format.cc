#include <float.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <algorithm>

// #include <omp.h>
#include <pcl/common/time.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>

#include "utils/cmd_parser.h"
#include "io/off_io.h"
// #include "utils/eigen3_utils.h"
// #include "utils/pcl_utils.h"
// #include "utils/shading_utils.h"


int main(int argc, char** argv) {
  pcl::StopWatch watch;

  // Parse cmd flags
  std::map<std::string, std::string> flags;
  parseCmdArgs(argc, argv, &flags);
  bool arg_error = false;

  std::string model_filename, output_filename;
  if (!(getFlag(flags, "model", "",  &model_filename) &&
        getFlag(flags, "output", "",  &output_filename))) {
    printf("Error: specify --model and --output\n");
    arg_error = true;
  }
  if (arg_error) return 1;

  pcl::PolygonMesh mesh;
  if (!pcl::io::loadPolygonFile(model_filename, mesh)) {
    printf("Error: could not load model from %s.\n", model_filename.c_str());
    return 1;
  }

  if (model_filename.find(".pcd") != std::string::npos) {
    pcl::PointCloud<pcl::PointWithViewpoint>::Ptr cloud (
      new pcl::PointCloud<pcl::PointWithViewpoint>);
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud,*cloud, indices);

    if (output_filename.find(".pcd") != std::string::npos) {
      pcl::PCDWriter writer;
      // PCL_INFO ("Wrote %lu points (%d x %d) to %s\n", cloud->points.size (), cloud->width, cloud->height, output_filename.c_str ());
      writer.writeBinaryCompressed (output_filename.c_str (), *cloud);
    } else {
      // pcl::PCLPointCloud2 xyz_cloud, current_cloud = mesh.cloud;
      // pcl::toPCLPointCloud2(*points, xyz_cloud);
      // pcl::concatenateFields(xyz_cloud, current_cloud, mesh.cloud);
      pcl::toPCLPointCloud2(*cloud, mesh.cloud);
      pcl::io::savePolygonFile(output_filename, mesh);
    }

  } else
    pcl::io::savePolygonFile(output_filename, mesh);
  return 0;
}