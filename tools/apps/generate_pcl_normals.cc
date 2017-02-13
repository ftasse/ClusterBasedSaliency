#include <float.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <algorithm>

// #include <omp.h>
#include <pcl/common/time.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>

#include "utils/cmd_parser.h"
#include "features/normals.h"
#include "io/off_io.h"
#include "utils/eigen3_utils.h"
#include "utils/pcl_utils.h"
#include "utils/shading_utils.h"


void generate_normals(pcl::PolygonMesh& mesh, std::size_t neighbourhood_size, bool use_topology) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr points (
    new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals (
    new pcl::PointCloud<pcl::Normal>);
  pcl::fromPCLPointCloud2(mesh.cloud, *points);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  computeNormalsIfNecessary(mesh, points, normals, neighbourhood_size, tree, use_topology, true);
}

int main(int argc, char** argv) {
  pcl::StopWatch watch;

  // Parse cmd flags
  std::map<std::string, std::string> flags;
  parseCmdArgs(argc, argv, &flags);
  bool arg_error = false;

  std::string model_filename, model_filelist;
  std::string output_filelist, output_prefix;
  std::string output_suffix = ".vtk";
  int neighbourhood_size;
  bool use_topology;
  if (!(getFlag(flags, "model", "",  &model_filename) ^
        getFlag(flags, "model_filelist", "",  &model_filelist))) {
    printf("Error: specify --model xor --model_filelist\n");
    arg_error = true;
  }
  if (model_filelist.size() > 0 &&
      !getFlag(flags, "output_filelist", "",  &output_filelist)) {
    printf("Error: specify --output_filelist\n");
    arg_error = true;
  }
  getFlag(flags, "output_prefix", "",  &output_prefix);
  getFlag(flags, "neighbourhood_size", 18,  &neighbourhood_size);
  getFlag(flags, "use_topology", true, &use_topology);
  if (arg_error) return 1;

  if (model_filename.size() > 0) {
    pcl::PolygonMesh mesh;
    if (!pcl::io::loadPolygonFile(model_filename, mesh)) {
      printf("Error: could not load model from %s.\n", model_filename.c_str());
      return 1;
    }
    generate_normals(mesh, neighbourhood_size, use_topology);
    std::string output_filename = 
      generate_output_filename(model_filename, output_suffix, "", output_prefix);
    pcl::io::savePolygonFile(output_filename, mesh);
  }

  if  (model_filelist.size() > 0) {
    std::ifstream model_filelist_in (model_filelist.c_str());
    std::ofstream output_filelist_out (output_filelist.c_str());

    while (!model_filelist_in.eof()) {
      std::string model_id, filename;
      model_filelist_in >> model_id >> filename;
      if (model_id.size() == 0 && filename.size() == 0) continue;

      pcl::PolygonMesh mesh;
      if (!pcl::io::loadPolygonFile(filename, mesh)) {
        printf("Error: could not load model from %s.\n", filename.c_str());
        continue;
      }
      generate_normals(mesh, neighbourhood_size, use_topology);
      std::string output_filename = 
        generate_output_filename(filename, output_suffix, model_id, output_prefix);
      pcl::io::savePolygonFile(output_filename, mesh);
      output_filelist_out << model_id << " " << output_filename << "\n";
    }

    model_filelist_in.close();
    output_filelist_out.close();
  }

  return 0;
}