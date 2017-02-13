#ifndef TOOLS_SHADINGUTILS_H
#define TOOLS_SHADINGUTILS_H

#include <pcl/point_cloud.h>

namespace pcl {
  class Normal;
  class PointXYZ;
  class RGB;
  class PCLPointCloud2;

namespace OpenFlipperPropertyInfo {
  enum EntityFilter {
    EF_ANY = 0xFF, EF_FACE = 0x01, EF_EDGE = 0x02, EF_HALFEDGE = 0x04,
    EF_VERTEX = 0x08, EF_HALFFACE = 0x10, EF_CELL = 0x20
  };
}

void convertNormalsToRGBs(pcl::PointCloud<pcl::Normal>::ConstPtr normals,
			  pcl::PointCloud<pcl::RGB>::Ptr colors);

void convertScalarFctToRGBs(
  const std::vector<float>& values, 
  pcl::PointCloud<pcl::RGB>::Ptr colors, size_t nb_points, float range = -1);

void concatenatePointsNormalsRGBs(pcl::PointCloud<pcl::PointXYZ>::Ptr points,
				   pcl::PointCloud<pcl::Normal>::Ptr normals,
				   pcl::PointCloud<pcl::RGB>::Ptr colors,
				   pcl::PCLPointCloud2& cloud);

void createPointCloudFromKeyPoint( 
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
  const std::vector<int> &keypoint_indices,
  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoint_cloud);

bool saveAsOpenFlipperProperty(
  const std::string& filename,
  const std::vector<int>& property,
  const std::string& property_name,
  OpenFlipperPropertyInfo::EntityFilter entity_filter);

bool saveAsOpenFlipperProperty(
  const std::string& filename,
  const Eigen::MatrixXf& property,
  const std::string& property_name,
  OpenFlipperPropertyInfo::EntityFilter entity_filter);

} // pcl

#endif // TOOLS_SHADINGUTILS_H
