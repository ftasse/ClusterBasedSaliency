#include <float.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <algorithm>

#include <pcl/common/time.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include "tools/utils/cmd_parser.h"
#include "tools/features/normals.h"
#include "tools/io/off_io.h"
#include "tools/utils/eigen3_utils.h"
#include "tools/utils/pcl_utils.h"
#include "tools/utils/shading_utils.h"

#include "saliency/clustering_saliency_detection.h"
#include "saliency/saliency_output_details.h"

void generate_saliency(
    pcl::PolygonMesh& mesh, float& neighbourhood_radius_percent,
    size_t num_segments, bool compute_per_point, bool use_adaptive_clustering,
    float distribution_importance, float clustering_fpfh_importance,
    std::vector<float>& saliency, 
    ClusterBasedSaliencyDetection::IntermediateData* intermediate_data = NULL,
    bool normals_from_topology = true, bool persist_fpfhs = false, bool verbose  = false, 
    std::string save_fpfhs_filename = "") {
  pcl::StopWatch watch;
  float descriptor_computation_time = 0;

  pcl::PointCloud<pcl::PointXYZ>::Ptr points (
    new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals (
    new pcl::PointCloud<pcl::Normal>);
  pcl::fromPCLPointCloud2(mesh.cloud, *points);
  float scale_factor = getNormalizingScaleFactor(points, false);

  // Build kdtree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (points);
  if (verbose)
    pcl::console::print_highlight ("Built KdTree. Time: %f\n", watch.getTimeSeconds());

  // Get average spacing
  float neighbourhood_radius = neighbourhood_radius_percent/scale_factor;
  size_t neighbourhood_size = 18;
  // if (points->size() < 1000000) 
  {
    while (true) {
      neighbourhood_size = computeAverageDensity(points, neighbourhood_radius, tree);
      std::cout << "neighbourhood_size: " << neighbourhood_size << "\n";
      if (neighbourhood_size >= 6) break;
      neighbourhood_radius_percent*=1.1;
      neighbourhood_radius = neighbourhood_radius_percent/scale_factor;
    }
  }
  bool use_radius_search = true;
  if (mesh.polygons.size() > 50000) 
  {
    std::vector<int> nn_indices;
    std::vector<float> nn_dists;
    for (size_t i = 0; i < points->size(); ++i) {
      if (tree->radiusSearch(i, neighbourhood_radius, nn_indices, nn_dists) < 4) {
        use_radius_search = false; break;
      }
    }
  } else use_radius_search = false;
  pcl::console::print_highlight ("Computed neighbourhood radius. %f %zd %d. Time: %f\n", 
    neighbourhood_radius, neighbourhood_size, use_radius_search, watch.getTimeSeconds());

  // Get normals - Note: normals affect fpfhs and thus saliency detection
  if (pcl::cloudHasNormals(mesh.cloud)) pcl::fromPCLPointCloud2(mesh.cloud, *normals);
  else {
    if (verbose) printf("Computing normals\n");
    if (mesh.polygons.size() > 50000) 
      computeNormalsIfNecessary(
        mesh, points, normals, std::min(neighbourhood_size, (size_t)18),
        tree, normals_from_topology);
    else {
      if (use_radius_search) computeNormals(points, normals, neighbourhood_radius, tree);
      else computeNormals(points, normals, neighbourhood_size, tree);
      computeAndOrientNormalsWithCGAL(points, normals, std::min(neighbourhood_size, (size_t) 30), true);
    }
    pcl::PCLPointCloud2 normals_cloud, current_cloud = mesh.cloud;
    pcl::toPCLPointCloud2(*normals, normals_cloud);
    pcl::concatenateFields(normals_cloud, current_cloud, mesh.cloud);
  }

  if (verbose)
    pcl::console::print_highlight ("Computed Normals. Time: %f\n", watch.getTimeSeconds());
  
  // Get FPFHS
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (
      new pcl::PointCloud<pcl::FPFHSignature33>);
  if (pcl::getFieldsList(mesh.cloud).find("fpfh") == std::string::npos) {
    if (verbose) printf("Computing fpfhs\n");
    computeCustomFPFHs(points, normals, fpfhs, use_radius_search?0:(neighbourhood_size*1.2), neighbourhood_radius*1.2f, tree);
    // computeFPFHs(points, normals, fpfhs, use_radius_search?0:(neighbourhood_size*1.2), neighbourhood_radius*1.2f, tree);
    if (persist_fpfhs) {
      pcl::PCLPointCloud2 fpfhs_cloud, current_cloud = mesh.cloud;
      pcl::toPCLPointCloud2(*fpfhs, fpfhs_cloud);
      pcl::concatenateFields(fpfhs_cloud, current_cloud, mesh.cloud);
    }
    if (save_fpfhs_filename.size() != 0) {
      pcl::PCDWriter w;
      w.writeBinaryCompressed(save_fpfhs_filename, mesh.cloud);
    }
  } else {
    pcl::fromPCLPointCloud2(mesh.cloud, *fpfhs);
  }
  descriptor_computation_time = watch.getTimeSeconds();

  if (intermediate_data)
    intermediate_data->descriptor_computation_time = descriptor_computation_time;
  if (verbose)
    pcl::console::print_highlight ("Computed FPFHs. Time: %f\n", descriptor_computation_time);

  // Compute saliency
  ClusterBasedSaliencyDetection saliency_detection;
  saliency_detection.setInputCloud(points);
  saliency_detection.setInputNormals (normals);
  saliency_detection.setInputHistograms (fpfhs);
  saliency_detection.setSearchMethod(tree);
  saliency_detection.setClusteringResolution(neighbourhood_radius);
  saliency_detection.setClusterSize(num_segments);
  saliency_detection.useAdaptiveClustering(use_adaptive_clustering);
  saliency_detection.setDistributionImportance(distribution_importance);
  saliency_detection.setClusteringFPFHImportance(clustering_fpfh_importance);
  saliency_detection.setRadiusSearch(neighbourhood_radius);
  saliency_detection.computeSaliency(saliency, compute_per_point, intermediate_data);
  if (verbose)
    pcl::console::print_highlight ("Computed Saliency. Time: %f\n", watch.getTimeSeconds());
}

void generate_surface_variation(
    pcl::PolygonMesh& mesh, float neighbourhood_radius_percent,
    std::vector<float>& surface_variation)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr points (
    new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals (
    new pcl::PointCloud<pcl::Normal>);
  pcl::fromPCLPointCloud2(mesh.cloud, *points);
  pcl::fromPCLPointCloud2(mesh.cloud, *normals);

  float scale_factor = getNormalizingScaleFactor(points, false);
  float neighbourhood_radius = neighbourhood_radius_percent/scale_factor;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (points);
  computeSurfaceVariation(
      points, normals, surface_variation, neighbourhood_radius, tree);
}

void generate_fpfh_pca(
    pcl::PolygonMesh& mesh, float neighbourhood_radius_percent,
    std::vector<float>& fpfh_pca)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr points (
    new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals (
    new pcl::PointCloud<pcl::Normal>);
  pcl::fromPCLPointCloud2(mesh.cloud, *points);
  pcl::fromPCLPointCloud2(mesh.cloud, *normals);
  
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (
    new pcl::PointCloud<pcl::FPFHSignature33>);
  pcl::fromPCLPointCloud2(mesh.cloud, *fpfhs);

  Eigen::MatrixXf fpfhs_matrix;
  createMatrixFromFPFHS(fpfhs, fpfhs_matrix);
  reduceDimension(fpfhs_matrix, 1);
  fpfhs_matrix = -fpfhs_matrix;
  fpfh_pca = std::vector<float> (
    fpfhs_matrix.data(), fpfhs_matrix.data()+points->size());
}

void saveScalarAsColor(
    const pcl::PolygonMesh& mesh, std::vector<float>& scalar,
    std::string scalar_type, std::string prefix) 
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr points (
    new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals (
    new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::RGB>::Ptr colors (
    new pcl::PointCloud<pcl::RGB>);
  pcl::fromPCLPointCloud2(mesh.cloud, *points);
  pcl::fromPCLPointCloud2(mesh.cloud, *normals);
  convertScalarFctToRGBs(scalar, colors, points->size());
  pcl::PolygonMesh colored_mesh = mesh;
  concatenatePointsNormalsRGBs(points, normals, colors, colored_mesh.cloud); 
  pcl::io::savePolygonFile((prefix + "_" + scalar_type + ".ply").c_str(), colored_mesh);
  pcl::saveAsOpenFlipperProperty(
      prefix + "_" + scalar_type + ".vprop",
      Eigen::Map<Eigen::VectorXf>(&scalar[0], scalar.size()),
      "v:" + scalar_type, pcl::OpenFlipperPropertyInfo::EF_VERTEX);
}

int main(int argc, char **argv) {
  // Parse cmd flags
  std::map<std::string, std::string> flags;
  parseCmdArgs(argc, argv, &flags);
  bool arg_error = false;

  std::string model_filename, model_filelist;
  std::string output_filelist, output_prefix;
  bool save_fpfhs;
  size_t num_segments;
  float neighbourhood_radius_percent;
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
  getFlag(flags, "radius_percent", 0.01f,  &neighbourhood_radius_percent);
  getFlag(flags, "num_segments", (size_t) 500,  &num_segments);
  
  bool save_intermediate, verbose, normals_from_topology;
  bool save_surface_variation, save_fpfh_pca;
  bool compute_per_point, use_adaptive_clustering;
  float  distribution_importance, clustering_fpfh_importance;
  getFlag(flags, "verbose", false, &verbose);
  getFlag(flags, "normals_from_topology", true, &normals_from_topology);
  getFlag(flags, "save_intermediate", false, &save_intermediate);
  getFlag(flags, "save_surface_variation", false, &save_surface_variation);
  getFlag(flags, "save_fpfh_pca", false, &save_fpfh_pca);
  getFlag(flags, "save_fpfhs", false, &save_fpfhs);
  getFlag(flags, "compute_per_point", true, &compute_per_point);
  getFlag(flags, "use_adaptive_clustering", true, &use_adaptive_clustering);
  getFlag(flags, "distribution_importance", 0.5f, &distribution_importance);
  getFlag(flags, "clustering_fpfh_importance", 2.0f, &clustering_fpfh_importance);

  if (arg_error)   return 1;
  
  if (model_filename.size() > 0) {
    std::string dir, basename;
    get_dir_and_basename(model_filename, dir, basename);
    char *tmp_filename = tmpnam(NULL);
    model_filelist = tmp_filename;
    std::ofstream ofs (model_filelist.c_str());
    ofs << basename << " " << model_filename << "\n";
    ofs.close();
    printf("tmp file: %s\n", tmp_filename);
  }

  std::ifstream filelist_in (model_filelist.c_str());
  std::ofstream filelist_out(output_filelist.c_str());

  while (!filelist_in.eof()) {
    std::string model_id, filename;
    filelist_in >> model_id >> filename;
    if (model_id.size() == 0 && filename.size() == 0) continue;
      
    pcl::StopWatch watch;
    pcl::PolygonMesh mesh;
    if (verbose)
      pcl::console::print_highlight ("Process %s: %s\n", model_id.c_str(), filename.c_str());
    if (!pcl::io::loadPolygonFile(filename, mesh)) {
      printf("Error: could not load model from %s.\n", filename.c_str());
      continue;
    }
    if (verbose)
      pcl::console::print_highlight ("Loaded mesh. Time: %f\n", watch.getTimeSeconds());

    watch.reset();
    std::vector<float> saliency;
    ClusterBasedSaliencyDetection::IntermediateData* intermediate_data = NULL;
    if (save_intermediate)
      intermediate_data = new ClusterBasedSaliencyDetection::IntermediateData;
    float radius_percent = neighbourhood_radius_percent;
    generate_saliency(mesh, radius_percent, num_segments, 
                      compute_per_point, use_adaptive_clustering,
                      distribution_importance, clustering_fpfh_importance,
                      saliency, intermediate_data, normals_from_topology,
                      save_fpfh_pca || save_fpfhs, verbose,
                      save_fpfhs ? output_prefix + model_id + ".pcd" : "");
    saveScalarAsColor(mesh, saliency, "saliency", output_prefix + model_id);
    if (output_filelist.size() > 0)
      filelist_out << model_id << " " << output_prefix + model_id + "_saliency.vprop\n";

    if (save_surface_variation) {
      std::vector<float> surface_variation;
      generate_surface_variation(mesh, radius_percent, surface_variation);
      saveScalarAsColor(mesh, surface_variation, "surface_variation", output_prefix + model_id);
    }

    if (save_fpfh_pca) {
      std::vector<float> fpfh_pca;
      generate_fpfh_pca(mesh, radius_percent, fpfh_pca);
      saveScalarAsColor(mesh, fpfh_pca, "fpfh_pca", output_prefix + model_id);
    }

    if (intermediate_data && save_intermediate) {
      if (verbose) std::cout << "Save intermediate results ...\n";
      saveSaliencyIntermediateResults(mesh, *intermediate_data, output_prefix + model_id);
      delete intermediate_data;
    }
  }

  filelist_in.close();
  filelist_out.close();

  if (model_filename.size() > 0) {
    // remove temporary filelist 
    remove(model_filelist.c_str());
  }

  return 0;
}
