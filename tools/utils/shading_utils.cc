#include "utils/shading_utils.h"

#include <fstream>
#include <pcl/conversions.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include "utils/color_gradient.h"

namespace pcl {

void convertNormalsToRGBs(pcl::PointCloud<pcl::Normal>::ConstPtr normals,
			  pcl::PointCloud<pcl::RGB>::Ptr colors) {
  colors->points.resize(normals->points.size());
  colors->width = normals->width;
  colors->height = normals->height;
  for (size_t i = 0; i < normals->points.size(); ++i) {
    colors->points[i].r = (normals->points[i].normal_x+1.0)*255/2.0;
    colors->points[i].g = (normals->points[i].normal_y+1.0)*255/2.0;
    colors->points[i].b = (normals->points[i].normal_z+1.0)*255/2.0;
    /*if (i == 0) printf("%f %f %f / %d %d %d\n", 
		       normals->points[i].normal_x, normals->points[i].normal_y, normals->points[i].normal_z,
		       colors->points[i].r, colors->points[i].g, colors->points[i].b);*/
  }
}

void concatenatePointsNormalsRGBs(pcl::PointCloud<pcl::PointXYZ>::Ptr points,
				  pcl::PointCloud<pcl::Normal>::Ptr normals,
				  pcl::PointCloud<pcl::RGB>::Ptr colors,
				  pcl::PCLPointCloud2& cloud) {
  pcl::PCLPointCloud2 colors_cloud, normals_cloud, points_cloud, concatenated;
  pcl::toPCLPointCloud2(*colors, colors_cloud);
  pcl::toPCLPointCloud2(*normals, normals_cloud);
  pcl::toPCLPointCloud2(*points, cloud);
  pcl::concatenateFields(colors_cloud, cloud, concatenated);
  pcl::concatenateFields(normals_cloud, concatenated, cloud);
}

void convertScalarFctToRGBs(
    const std::vector<float>& values, 
    pcl::PointCloud<pcl::RGB>::Ptr colors, 
    size_t nb_points,
    float range) {
  assert(values.size() == nb_points);
  colors->points.resize(nb_points);
  colors->width = nb_points;
  colors->height = 1;

  float min_val = *std::min_element(values.begin(), values.end());
  float max_val = *std::max_element(values.begin(), values.end());
  if (range <= 0) range = (max_val - min_val);
  if (range <= 0) return;

  ColorGradient heatmap_gradient; 
  heatmap_gradient.createDefaultHeatMapGradient();

  for (size_t i = 0; i < nb_points; ++i) {
    pcl::RGB color;
    float t = (values[i] - min_val)/range;
    heatmap_gradient.getColorAtValue(t, color.r, color.g, color.b);
    colors->points[i] = color;
  }
}

void createPointCloudFromKeyPoint( 
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
  const std::vector<int> &keypoint_indices,
  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoint_cloud) {
  keypoint_cloud->points.resize(keypoint_indices.size());
  for (size_t k = 0; k < keypoint_indices.size(); ++k) {
    keypoint_cloud->points[k] = (points->points[keypoint_indices[k]]);
  }
}

bool saveAsOpenFlipperProperty(
    const std::string& filename,
    const std::vector<int>& property,
    const std::string& property_name,
    OpenFlipperPropertyInfo::EntityFilter entity_filter) {
    std::ofstream ofs(filename.c_str());
  if (!ofs.is_open())  return false;
  std::string version = "1";
  int num_values = property.size();

  ofs << version << ", " << num_values << ", " << entity_filter << ", "
      << "int" << ", " << property_name << "\n";
  for (size_t i = 0; i < num_values; ++i) ofs << property[i] << "\n";
  ofs.close();
  return true;
}

bool saveAsOpenFlipperProperty(
    const std::string& filename,
    const Eigen::MatrixXf& property,
    const std::string& property_name,
    OpenFlipperPropertyInfo::EntityFilter entity_filter) {
  assert(property.cols() == 1 || property.cols() == 3);

  std::ofstream ofs(filename.c_str());
  if (!ofs.is_open())  return false;
  std::string version = "1";
  int num_values = property.rows();
  std::string property_type;
  if (property.cols() == 1) property_type = "double";
  else property_type = "Vec3d";

  // create header
  ofs << version << ", " << num_values << ", " << entity_filter << ", "
      << property_type << ", " << property_name << "\n";

  // save data
  if (property.cols () == 3) {
    for (size_t i = 0; i < num_values; ++i)
        ofs << "( " << property(i, 0) << " " << property(i, 1) << property(i, 2) << " )\n";
  } else
      ofs << property << "\n";

  ofs.close();
  return true;
}

} // pcl
