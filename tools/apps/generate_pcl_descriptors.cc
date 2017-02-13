#include <float.h>
#include <stdlib.h>
#include <fstream>

#include <pcl/io/ply_io.h>

#include "features/normals.h"
#include "features/descriptors.h"
#include "utils/cmd_parser.h"
#include "utils/pcl_utils.h"
#include "io/off_io.h"
#include "utils/matrix_io.h"

int main(int argc, char **argv) {
  // Parse cmd flags
  std::map<std::string, std::string> flags;
  parseCmdArgs(argc, argv, &flags);
  bool arg_error = false;

  std::string model_filename, model_filelist;
  std::string keypoints_filename, keypoints_filelist;
  std::string output_filelist, output_prefix;
  size_t num_levels;
  float radius_percent;
  std::string descriptor_type;
  float epsilon;
  if (!(getFlag(flags, "model", "",  &model_filename) ^
        getFlag(flags, "model_filelist", "",  &model_filelist))) {
    printf("Error: specify --model xor --model_filelist\n");
    arg_error = true;
  }
  if (model_filelist.size() > 0 &&
      !(getFlag(flags, "output_filelist", "",  &output_filelist) && 
        getFlag(flags, "keypoints_filelist", "", &keypoints_filelist))) {
    printf("Error: specify --keypoints_filelist and --output_filelist\n");
    arg_error = true;
  }
  if (model_filename.size() > 0 && 
      !getFlag(flags, "keypoints", "", &keypoints_filename)) {
    printf("Error: specify --keypoints\n");
    arg_error = true;
  }
  getFlag(flags, "output_prefix", "",  &output_prefix);
  getFlag(flags, "radius_percent", 0.1f,  &radius_percent);
  getFlag(flags, "descriptor_type", "",  &descriptor_type);

  bool verbose, normals_from_topology;
  getFlag(flags, "verbose", false, &verbose);
  getFlag(flags, "normals_from_topology", true, &normals_from_topology);

  if (arg_error)   return 1;

  std::map<std::string, std::string> id_to_model_filenames;
  std::vector<std::pair<std::string, std::string> > id_to_keypoints_filenames;
  if (model_filename.size() > 0 && keypoints_filename.size() > 0) {
    getMapFromFilename(model_filename, id_to_model_filenames);
    id_to_keypoints_filenames.push_back(
      std::make_pair(id_to_model_filenames.begin()->first, keypoints_filename));
  } else if (model_filelist.size() > 0 && keypoints_filelist.size() > 0) {
    getMapFromFilelist(model_filelist, id_to_model_filenames);
    getMapFromFilelist(keypoints_filelist, id_to_keypoints_filenames);
  }

  std::vector<std::pair<std::string, std::string> >::iterator keypoints_filenames_it;
  std::map<std::string, std::string>::iterator model_filename_it; 
  keypoints_filenames_it =  id_to_keypoints_filenames.begin();

  std::ofstream filelist_out(output_filelist.c_str());

  for  (; keypoints_filenames_it != id_to_keypoints_filenames.end(); ++keypoints_filenames_it) {
    if (verbose)
      pcl::console::print_highlight ("Process %s: %s\n", 
        keypoints_filenames_it->first.c_str(), keypoints_filenames_it->second.c_str());
    pcl::PolygonMesh mesh, keypoints_mesh;
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoint_coords (new pcl::PointCloud<pcl::PointXYZ>);

    std::string model_id = keypoints_filenames_it->first;
    model_filename_it = id_to_model_filenames.find(model_id);
    if (model_filename_it == id_to_model_filenames.end()){
      printf("Error: could not find model filename corresponding to: %s\n", model_id.c_str());
      continue;
    }
    if (!pcl::io::loadPolygonFile(keypoints_filenames_it->second, keypoints_mesh)) {
      printf("Error: could not load keypoints from: %s\n", keypoints_filenames_it->second.c_str());
      continue;
    } else {
      pcl::fromPCLPointCloud2(keypoints_mesh.cloud, *keypoint_coords);
    }
    if (!pcl::io::loadPolygonFile(model_filename_it->second, mesh)) {
      printf("Error: could not load model from %s.\n", model_filename_it->second.c_str());
      continue;
    }

    Eigen::MatrixXf descriptors;
    computeKeyPointDescriptors(mesh, keypoint_coords,
      radius_percent, descriptor_type, descriptors, normals_from_topology, verbose);
    std::stringstream filename_ss;
    filename_ss << output_prefix << model_id << "_" << descriptor_type << ".txt";
    
    std::string descriptor_filename = filename_ss.str();
    saveEigenMatrix(descriptor_filename, descriptors);
    if (output_filelist.size() > 0)
      filelist_out << model_id << " " << descriptor_filename << "\n";
  }

  filelist_out.close();
  return 0;
}
