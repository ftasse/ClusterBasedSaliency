#include <float.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <random>

// #include <omp.h>
#include <pcl/common/time.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include "utils/cmd_parser.h"
#include "features/normals.h"
#include "io/off_io.h"
#include "utils/pcl_utils.h"

void generate_sampling(
  pcl::PolygonMesh& mesh, pcl::PointCloud<pcl::PointNormal>::Ptr sampled_points, size_t num_of_keypoints, bool use_topology, bool apply_lloyd, bool verbose) 
{
  pcl::PointCloud<pcl::PointNormal>::Ptr point_normals(new pcl::PointCloud<pcl::PointNormal>);
  pcl::fromPCLPointCloud2(mesh.cloud, *point_normals);

  pcl::ModelSampler sampler(mesh, std::vector<float>(), use_topology);
  while (sampled_points->size() < num_of_keypoints) {
    sampled_points->points.push_back(sampler.getRandomPoint());
  }

  if (apply_lloyd)
    pcl::applyLloydRelaxation<pcl::PointNormal>(point_normals, sampled_points);
}

bool saveOFFFile(const std::string& filename, const pcl::PointCloud<pcl::PointNormal>& sampled_points) {
  std::ofstream ofs(filename.c_str());
  if (!ofs.is_open()) return false;

  ofs << "OFF\n";
  ofs << sampled_points.size() << " 0 0\n";

  for (size_t i=0; i < sampled_points.size(); ++i) {
    pcl::PointNormal point = sampled_points.points[i];
    ofs << point.x << " " << point.y << " " << point.z << "\n";
  }

  ofs.close();
  return true;
}


int main (int argc, char** argv) {
  // Parse cmd flags
  std::map<std::string, std::string> flags;
  parseCmdArgs(argc, argv, &flags);
  bool arg_error = false;

  std::string model_filename, model_filelist;
  std::string output_filelist, output_prefix;
  size_t num_of_keypoints;
  bool verbose, use_topology, apply_lloyd;
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
  if (!getFlag(flags, "output_prefix", "",  &output_prefix)) {
    printf("Error: specify --output_prefix\n");
    arg_error = true;
  }
  getFlag(flags, "num_of_keypoints", (size_t) 500,  &num_of_keypoints);
  getFlag(flags, "use_topology", true,  &use_topology);
  getFlag(flags, "apply_lloyd", false, &apply_lloyd);
  getFlag(flags, "verbose", false,  &verbose);

  std::string output_suffix = "_keypoints.off";

  if (arg_error) return 1;

  if (model_filename.size() > 0) {
    pcl::PolygonMesh mesh;
    if (!pcl::io::loadPolygonFile(model_filename, mesh)) {
      printf("Error: could not load model from %s.\n", model_filename.c_str());
      return 1;
    }
    pcl::PointCloud<pcl::PointNormal>::Ptr sampled_points (new pcl::PointCloud<pcl::PointNormal>);
    generate_sampling(mesh, sampled_points, num_of_keypoints, use_topology, apply_lloyd, verbose);
    std::string output_filename = 
      generate_output_filename(model_filename, output_suffix, "", output_prefix);
    saveOFFFile(output_filename, *sampled_points);
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
      if (verbose) printf("Processing: %s %s %d\n", model_id.c_str(), filename.c_str(), num_of_keypoints);
      pcl::PointCloud<pcl::PointNormal>::Ptr sampled_points (new pcl::PointCloud<pcl::PointNormal>);
      generate_sampling(mesh, sampled_points, num_of_keypoints, use_topology, apply_lloyd, verbose);
      std::string output_filename = 
        generate_output_filename(filename, output_suffix, model_id, output_prefix);
      saveOFFFile(output_filename, *sampled_points);
      output_filelist_out << model_id << " " << output_filename << "\n";
    }

    model_filelist_in.close();
    output_filelist_out.close();
  }
}