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
#include <pcl/surface/mls.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/point_types.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/gp3.h>

#include "utils/cmd_parser.h"
#include "features/normals.h"
#include "io/off_io.h"
#include "utils/pcl_utils.h"

void generate_sampling(
  pcl::PolygonMesh& mesh, pcl::PointCloud<pcl::PointNormal>::Ptr sampled_points, 
  float grid_leaf_size, size_t point_density, int meshed, bool unit_scale, bool verbose) 
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  //if (!pcl::cloudHasNormals(mesh.cloud)) 
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr points (
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(mesh.cloud, *points);
    float scale_factor = getNormalizingScaleFactor(points, unit_scale);
    if (!unit_scale) grid_leaf_size /= scale_factor;

    if (!pcl::cloudHasNormals(mesh.cloud)) {
      if (verbose) printf("Computing normals\n");
      pcl::PointCloud<pcl::Normal>::Ptr normals (
        new pcl::PointCloud<pcl::Normal>);
      computeNormalsIfNecessary(mesh, points, normals, 18, tree, true, true);
    }
  }

  if (meshed == 1) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_normals(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(mesh.cloud, *point_normals);
    typedef pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> MLS;
    MLS mls;
    mls.setComputeNormals (true);
    mls.setInputCloud (point_normals);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (grid_leaf_size*2);
    // mls.setUpsamplingMethod(MLS::RANDOM_UNIFORM_DENSITY);
    // mls.setPointDensity(point_density);
    // mls.setUpsamplingMethod (MLS::VOXEL_GRID_DILATION); // (MLS::SAMPLE_LOCAL_PLANE); 
    // mls.setUpsamplingRadius (grid_leaf_size);
    // mls.setUpsamplingStepSize (grid_leaf_size/2);
    mls.process (*sampled_points);
    pcl::PointCloud<pcl::PointXYZ>::Ptr points (
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals (
        new pcl::PointCloud<pcl::Normal>);
    pcl::copyPointCloud (*sampled_points, *points);
    pcl::copyPointCloud (*sampled_points, *normals);
    computeAndOrientNormalsWithCGAL(points, normals, point_density, false);
    pcl::concatenateFields (*points, *normals, *sampled_points);
    return;
  }

  pcl::PointCloud<pcl::PointNormal>::Ptr point_normals(new pcl::PointCloud<pcl::PointNormal>);
  pcl::fromPCLPointCloud2(mesh.cloud, *point_normals);

  //Add more points
  size_t target_num_points = 100000;
  pcl::ModelSampler sampler(mesh);
  while (point_normals->size() < target_num_points) {
    point_normals->points.push_back(sampler.getRandomPoint());
  }
  // *sampled_points = *point_normals;
 
  pcl::PointCloud<int> sampled_indices;
  pcl::UniformSampling<pcl::PointNormal> uniform_sampling;
  if (meshed == 2) {
    if (verbose) std::cout << "Poisson reconstruction\n";
    uniform_sampling.setInputCloud (point_normals);
    uniform_sampling.setRadiusSearch (grid_leaf_size*2);
    uniform_sampling.compute (sampled_indices);
    pcl::PointCloud<pcl::PointNormal>::Ptr poisson_cloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::copyPointCloud (*point_normals, sampled_indices.points, *poisson_cloud);
    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setInputCloud(poisson_cloud);
    poisson.reconstruct (mesh);
    if (verbose) std::cout << "Poisson reconstruction done\n";
  } else {
    uniform_sampling.setInputCloud (point_normals);
    uniform_sampling.setRadiusSearch (grid_leaf_size);
    uniform_sampling.compute (sampled_indices);
    pcl::copyPointCloud (*point_normals, sampled_indices.points, *sampled_points);
    if (meshed == 3) {
      pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);
      tree->setInputCloud (sampled_points);
      pcl::toPCLPointCloud2(*sampled_points, mesh.cloud);

      pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
      gp3.setSearchRadius (grid_leaf_size*5);
      gp3.setMu (5);
      // gp3.setMinimumAngle(0);
      // gp3.setMaximumSurfaceAngle(2*M_PI);
      // gp3.setMaximumAngle(2*M_PI);
      gp3.setMaximumNearestNeighbors (200);
      // gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
      // gp3.setMinimumAngle(M_PI/18); // 10 degrees
      // gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
      gp3.setNormalConsistency(true);
      gp3.setConsistentVertexOrdering(true);
      gp3.setInputCloud (sampled_points);
      gp3.setSearchMethod (tree);
      gp3.reconstruct (mesh);
    }
  }
  return;
}

bool savePLYFile(const std::string& filename, const pcl::PointCloud<pcl::PointNormal>& sampled_points) {
  std::ofstream ofs(filename.c_str());
  if (!ofs.is_open()) return false;

  ofs << "ply\n";
  ofs << "format ascii 1.0\n";
  ofs << "comment PCL generated\n";
  ofs << "element vertex " << sampled_points.size() << "\n";
  ofs << "property float x\n";
  ofs << "property float y\n";
  ofs << "property float z\n";
  ofs << "property float nx\n";
  ofs << "property float ny\n";
  ofs << "property float nz\n";
  ofs << "property float curvature\n";
  ofs << "element face 0\n";
  ofs << "property list uchar int vertex_index\n";
  ofs << "end_header\n";

  for (size_t i=0; i < sampled_points.size(); ++i) {
    pcl::PointNormal point = sampled_points.points[i];
    ofs << point.x << " " << point.y << " " << point.z << " "
        << point.normal_x << " " << point.normal_y << " " << point.normal_z << " "
        << point.curvature << "\n";
  }

  ofs.close();
  return true;
}

bool saveVTKFile(const std::string& filename, const pcl::PointCloud<pcl::PointNormal>& sampled_points) {
  std::ofstream ofs(filename.c_str());
  if (!ofs.is_open()) return false;

  ofs << "# vtk DataFile Version 3.0\n";
  ofs << "vtk output\n";
  ofs << "ASCII\n";
  ofs << "DATASET POLYDATA\n";
  ofs << "POINTS " << sampled_points.size() << " float\n";
  for (size_t i=0; i < sampled_points.size(); ++i) {
    pcl::PointNormal point = sampled_points.points[i];
    ofs << point.x << " " << point.y << " " << point.z << "\n";
  }
  ofs << "POLYGONS 0 0\n";
  ofs << "POINT_DATA " <<  sampled_points.size() << "\n";
  ofs << "NORMALS Normals float\n";
  for (size_t i=0; i < sampled_points.size(); ++i) {
    pcl::PointNormal point = sampled_points.points[i];
    ofs << point.normal_x << " " << point.normal_y << " " << point.normal_z << "\n";
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
  float grid_leaf_size;
  size_t point_density;
  int meshed;
  bool unit_scale, verbose;
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
  getFlag(flags, "grid_leaf_size", 0.01f,  &grid_leaf_size);
  getFlag(flags, "point_density", (size_t) 9,  &point_density);
  getFlag(flags, "meshed", 0, &meshed);
  getFlag(flags, "unit_scale", true,  &unit_scale);
  getFlag(flags, "verbose", false,  &verbose);

  std::string output_suffix = (meshed <= 1) ? "_sampled.vtk" : "_sampled_mesh.ply";

  if (arg_error) return 1;

  if (model_filename.size() > 0) {
    pcl::PolygonMesh mesh;
    if (!pcl::io::loadPolygonFile(model_filename, mesh)) {
      printf("Error: could not load model from %s.\n", model_filename.c_str());
      return 1;
    }
    pcl::PointCloud<pcl::PointNormal>::Ptr sampled_points (new pcl::PointCloud<pcl::PointNormal>);
    generate_sampling(mesh, sampled_points, grid_leaf_size, point_density, meshed, unit_scale, verbose);
    std::string output_filename = 
      generate_output_filename(model_filename, output_suffix, "", output_prefix);
    if (!meshed) saveVTKFile(output_filename, *sampled_points);
    else pcl::io::savePolygonFile(output_filename, mesh);
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
      if (verbose) printf("Processing: %s %s %d\n", model_id.c_str(), filename.c_str(), meshed);
      pcl::PointCloud<pcl::PointNormal>::Ptr sampled_points (new pcl::PointCloud<pcl::PointNormal>);
      generate_sampling(mesh, sampled_points, grid_leaf_size, point_density, meshed, unit_scale, verbose);
      std::string output_filename = 
        generate_output_filename(filename, output_suffix, model_id, output_prefix);
      if (meshed <= 1) saveVTKFile(output_filename, *sampled_points);
      else pcl::io::savePolygonFile(output_filename, mesh);
      output_filelist_out << model_id << " " << output_filename << "\n";
    }

    model_filelist_in.close();
    output_filelist_out.close();
  }
}