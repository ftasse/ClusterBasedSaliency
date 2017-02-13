#ifndef SEGMENTATION_SEGMENTATION_H
#define SEGMENTATION_SEGMENTATION_H

#include <pcl/point_cloud.h>
#include <pcl/search/search.h>

class Segment;

namespace Eigen {
}

namespace pcl {
  class PointXYZ;
  class Normal;
  class FPFHSignature33;
  class RGB;

  void computeSuperClustersPapon(
      pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
      pcl::PointCloud<pcl::Normal>::ConstPtr normals,
      pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr fpfhs,
      std::vector<Segment>& segments, size_t nb_desired_clusters,
      float voxel_resolution, int nb_neighbours, float neighbourhood_radius, bool enforce_connectivity,
      float spatial_importance = 1.0, 
      float normal_importance = 1.0, 
      float fpfh_importance = 1.0);

  void computeClustersRegionGrowing(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& points,
      const pcl::PointCloud<pcl::Normal>::Ptr& normals,
      const pcl::search::Search<pcl::PointXYZ>::Ptr& tree,
      std::vector<Segment>& segments, 
      size_t nb_desired_clusters, size_t nb_neighbours);

  void computeSegmentAdjacencyWithNeighbourhood(
    const std::vector<Segment>& segments, 
		const pcl::search::Search<pcl::PointXYZ>::Ptr& tree,
    int nb_neighbours, float neighbourhood_radius,
    std::vector< std::vector<int> >& segment_adjacency);

  void classical_segment(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
    const pcl::search::Search<pcl::PointXYZ>::Ptr& tree,
    std::vector<Segment> &segments,
    int size_thres,
    int nb_neighbours, float neighbourhood_radius);

  void cleanup_segment(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
    const pcl::search::Search<pcl::PointXYZ>::Ptr& tree,
    int nb_neighbours, float neighbourhood_radius,
    std::vector<Segment> &segments,
    int size_thres);

  void convertSegmentsToRGBs(const std::vector<Segment>& segments,
         pcl::PointCloud<pcl::RGB>::Ptr colors,
         size_t nb_points);

  void convertSegmentsBoundariesToBlack(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
    const pcl::search::Search<pcl::PointXYZ>::Ptr& tree,
    const std::vector<int>& point_labels,
    pcl::PointCloud<pcl::RGB>::Ptr colors,
    size_t nb_points);

  void convertSegmentsBoundariesToBlack(
      pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
      const pcl::search::Search<pcl::PointXYZ>::Ptr& tree,
      const std::vector<Segment>& segments,
      pcl::PointCloud<pcl::RGB>::Ptr colors,
      size_t nb_points);

} // pcl

#endif // SEGMENTATION_SEGMENTATION_H
