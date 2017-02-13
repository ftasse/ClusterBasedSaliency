#include "saliency/saliency_output_details.h"

#include <pcl/conversions.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include "tools/utils/shading_utils.h"

void saveSaliencyIntermediateResults(
  const pcl::PolygonMesh &mesh,
  const ClusterBasedSaliencyDetection::IntermediateData &intermediate_data,
  std::string prefix) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr points (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
  pcl::fromPCLPointCloud2(mesh.cloud, *points);
  pcl::fromPCLPointCloud2(mesh.cloud, *normals);

  pcl::PointCloud<pcl::RGB>::Ptr colors (new pcl::PointCloud<pcl::RGB> ());
  colors->points.resize(points->size());
  colors->height = points->height; colors->width = points->width;

  pcl::PolygonMesh tmp_mesh;
  tmp_mesh.polygons = mesh.polygons;

  // Save segmentation
  if (intermediate_data.point_labels.size() > 0) {
    size_t num_clusters = 1 + *std::max_element(
      intermediate_data.point_labels.begin(), 
      intermediate_data.point_labels.end());

    std::vector<pcl::RGB> cluster_colors(num_clusters);
    for (size_t j = 0; j < cluster_colors.size(); ++j) {
      cluster_colors[j].r = rand()%255;
      cluster_colors[j].g = rand()%255;
      cluster_colors[j].b = rand()%255;
    }

    for (size_t i = 0; i< intermediate_data.point_labels.size(); ++i) {
      int label = intermediate_data.point_labels[i];
      if (label >= 0) {
        colors->points[i].r = cluster_colors[label].r;
        colors->points[i].g = cluster_colors[label].g;
        colors->points[i].b = cluster_colors[label].b;
      }
      else {
        pcl::console::print_warn("Point %d does not belong to a segment \n", i);
      }
    }
    concatenatePointsNormalsRGBs(points, normals, colors, tmp_mesh.cloud);
    char suffix[256];
    sprintf(suffix, "segmented_%zd", num_clusters);
    pcl::io::savePolygonFile((prefix+"_segmented.ply").c_str(), tmp_mesh);

    pcl::saveAsOpenFlipperProperty(
      prefix+"_segmented.vprop",
      intermediate_data.point_labels, std::string("v:") + suffix, 
      pcl::OpenFlipperPropertyInfo::EF_VERTEX);
  }

  // Save per-cluster weight
  if (intermediate_data.point_labels.size() > 0 && intermediate_data.cluster_weight.size() > 0) {
    std::vector<float> quality(intermediate_data.point_labels.size(), 0.0);
    for (size_t i = 0; i< intermediate_data.point_labels.size(); ++i) {
      int label = intermediate_data.point_labels[i];
      if (label >= 0) quality[i] = intermediate_data.cluster_weight[label];
    }

    convertScalarFctToRGBs(quality, colors, points->size());
    concatenatePointsNormalsRGBs(points, normals, colors, tmp_mesh.cloud);
    pcl::io::savePolygonFile((prefix+"_cluster_weight.ply").c_str(), tmp_mesh);
    pcl::saveAsOpenFlipperProperty(
      prefix + "_cluster_weight.vprop", 
      Eigen::Map<Eigen::VectorXf>(&quality[0], quality.size()), 
      "v:cluster_weight", pcl::OpenFlipperPropertyInfo::EF_VERTEX);
  }

  // Save per-cluster uniqueness
  if (intermediate_data.point_labels.size() > 0 && intermediate_data.cluster_uniqueness.size() > 0) {
    std::vector<float> quality(intermediate_data.point_labels.size(), 0.0);
    for (size_t i = 0; i< intermediate_data.point_labels.size(); ++i) {
      int label = intermediate_data.point_labels[i];
      if (label >= 0) quality[i] = intermediate_data.cluster_uniqueness[label];
    }

    convertScalarFctToRGBs(quality, colors, points->size());
    concatenatePointsNormalsRGBs(points, normals, colors, tmp_mesh.cloud);
    pcl::io::savePolygonFile((prefix+"_cluster_uniqueness.ply").c_str(), tmp_mesh);
    pcl::saveAsOpenFlipperProperty(
      prefix + "_cluster_uniqueness.vprop", 
      Eigen::Map<Eigen::VectorXf>(&quality[0], quality.size()), 
      "v:cluster_uniqueness", pcl::OpenFlipperPropertyInfo::EF_VERTEX);
  }

  // Save per-cluster distribution
  if (intermediate_data.point_labels.size() > 0 && intermediate_data.cluster_distribution.size() > 0) {
    std::vector<float> quality(intermediate_data.point_labels.size(), 0.0);
    for (size_t i = 0; i< intermediate_data.point_labels.size(); ++i)
    {
      int label = intermediate_data.point_labels[i];
      if (label >= 0) quality[i] = intermediate_data.cluster_distribution[label];
    }
    convertScalarFctToRGBs(quality, colors, points->size());
    concatenatePointsNormalsRGBs(points, normals, colors, tmp_mesh.cloud);
    pcl::io::savePolygonFile((prefix+"_cluster_distribution.ply").c_str(), tmp_mesh);
    pcl::saveAsOpenFlipperProperty(
      prefix + "_cluster_distribution.vprop", 
      Eigen::Map<Eigen::VectorXf>(&quality[0], quality.size()), 
      "v:cluster_distribution", pcl::OpenFlipperPropertyInfo::EF_VERTEX);
  }

  // Save per-cluster saliency
  if (intermediate_data.point_labels.size() > 0 && intermediate_data.cluster_saliency.size() > 0) {
    std::vector<float> quality(intermediate_data.point_labels.size(), 0.0);
    for (size_t i = 0; i< intermediate_data.point_labels.size(); ++i)
    {
      int label = intermediate_data.point_labels[i];
      if (label >= 0) quality[i] = intermediate_data.cluster_saliency[label];
    }
    convertScalarFctToRGBs(quality, colors, points->size());
    concatenatePointsNormalsRGBs(points, normals, colors, tmp_mesh.cloud);
    pcl::io::savePolygonFile((prefix+"_cluster_saliency.ply").c_str(), tmp_mesh);
    pcl::saveAsOpenFlipperProperty(
      prefix + "_cluster_saliency.vprop", 
      Eigen::Map<Eigen::VectorXf>(&quality[0], quality.size()), 
      "v:cluster_saliency", pcl::OpenFlipperPropertyInfo::EF_VERTEX);
  }

  // Save per-vertex normals
  convertNormalsToRGBs(normals, colors);
  concatenatePointsNormalsRGBs(points, normals, colors, tmp_mesh.cloud);  
  pcl::io::savePolygonFile((prefix+"_normals.ply").c_str(), tmp_mesh);

  // Save timing
  std::ofstream ofs((prefix + "_timings.csv").c_str());
  ofs << "descriptor_computation_time , " << intermediate_data.descriptor_computation_time << "\n";
  ofs << "clustering_time , " << intermediate_data.clustering_time << "\n";
  ofs << "probability_computation_time , " << intermediate_data.probability_computation_time << "\n";
  ofs << "clusterUD_computation_time , " << intermediate_data.clusterUD_computation_time << "\n";
  ofs << "cluster_saliency_time , " << intermediate_data.cluster_saliency_time << "\n";
  ofs << "saliency_assignment_time , " << intermediate_data.saliency_assignment_time << "\n";

  ofs << "\ntotal_time , " << intermediate_data.getTotalTimeInSecs() << "\n";
  ofs << "total_time , " << intermediate_data.total_time << "\n";
  ofs.close();
}