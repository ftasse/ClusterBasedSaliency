#ifndef TOOLS_FEATURES_DESCRIPTORS_H
#define TOOLS_FEATURES_DESCRIPTORS_H

#include <pcl/point_cloud.h>

namespace pcl {
  class FPFHSignature33;
  class Normal;
  class PointXYZ;
  class PCLPointCloud2;
  class PolygonMesh;

  void computeKeyPointDescriptors(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
    pcl::PointCloud<pcl::Normal>::ConstPtr normals,
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr keypoints_cloud,
    float radius, std::string descriptor_type,
    Eigen::MatrixXf& descriptors);

  void computeKeyPointDescriptors(
    pcl::PolygonMesh& mesh, pcl::PointCloud<pcl::PointXYZ>::Ptr keypoint_coords,
    float radius_percent, std::string descriptor_type, 
    Eigen::MatrixXf& descriptors, bool normals_from_topology = true, bool verbose  = false);
}

#endif // TOOLS_FEATURES_DESCRIPTORS_H