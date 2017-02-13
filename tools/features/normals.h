#ifndef TOOLS_FEATURES_NORMALS_H
#define TOOLS_FEATURES_NORMALS_H

#include <pcl/point_cloud.h>
#include <pcl/search/search.h>

namespace pcl {
  class FPFHSignature33;
  class Normal;
  class PointXYZ;
  class PCLPointCloud2;
  class PolygonMesh;
  class Vertices;

  bool cloudHasNormals(const pcl::PCLPointCloud2& cloud);

  void computeNormalsIfNecessary(pcl::PolygonMesh& mesh,
       pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
       pcl::PointCloud<pcl::Normal>::Ptr normals,
       size_t n_neighbours,
       const pcl::search::Search<pcl::PointXYZ>::Ptr& tree,
       bool use_topology = true, bool persist_normals = true);

  void computeNormals(pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
		      pcl::PointCloud<pcl::Normal>::Ptr normals,
		      float support_radius,
          const pcl::search::Search<pcl::PointXYZ>::Ptr& tree);

  void computeNormals(pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
        pcl::PointCloud<pcl::Normal>::Ptr normals,
        size_t nb_neighbours,
        const pcl::search::Search<pcl::PointXYZ>::Ptr& tree);

  void computeNormals(const std::vector<pcl::Vertices> &polygons,
		      pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
		      pcl::PointCloud<pcl::Normal>::Ptr normals);

  bool computeAndOrientNormalsWithCGAL(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
    pcl::PointCloud<pcl::Normal>::Ptr normals,
    size_t n_neighbours,
    bool only_orient = false);

  void computeFPFHs(pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
		    pcl::PointCloud<pcl::Normal>::ConstPtr normals,
		    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs,
		    size_t n_neighbours, float neighbourhood_radius,
        const pcl::search::Search<pcl::PointXYZ>::Ptr& tree);

  void computeCustomFPFHs(pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
        pcl::PointCloud<pcl::Normal>::ConstPtr normals,
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs,
        size_t n_neighbours, float neighbourhood_radius,
        const pcl::search::Search<pcl::PointXYZ>::Ptr& tree,
        boost::shared_ptr<std::vector<int> > indices = boost::shared_ptr<std::vector<int> >());

  void computeNormalsAndFPFHS(
    pcl::PolygonMesh& mesh,
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
    pcl::PointCloud<pcl::Normal>::Ptr normals,
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs,
    size_t n_neighbours, size_t n_descriptor_neighbours,
    const pcl::search::Search<pcl::PointXYZ>::Ptr& tree);

  void computeOrientedNormalsAndFPFHS(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
    pcl::PointCloud<pcl::Normal>::Ptr normals,
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs,
    size_t n_neighbours, size_t n_descriptor_neighbours,
    const pcl::search::Search<pcl::PointXYZ>::Ptr& tree);

  void computeNormalsAndFPFHS(
    pcl::PolygonMesh& mesh,
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
    pcl::PointCloud<pcl::Normal>::Ptr normals,
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs,
    float neighbourhood_radius, float descriptor_radius,
    const pcl::search::Search<pcl::PointXYZ>::Ptr& tree);

  void computeOrientedNormalsAndFPFHS(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
    pcl::PointCloud<pcl::Normal>::Ptr normals,
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs,
    float neighbourhood_radius, float descriptor_radius,
    const pcl::search::Search<pcl::PointXYZ>::Ptr& tree);

  void computeSurfaceVariation(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
    pcl::PointCloud<pcl::Normal>::Ptr normals,
    std::vector<float>& surface_variation,
    int nb_neighbours,
    const pcl::search::Search<pcl::PointXYZ>::Ptr& tree);

  void computeSurfaceVariation(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
    pcl::PointCloud<pcl::Normal>::Ptr normals,
    std::vector<float>& surface_variation,
    float neighbourhood_radius,
    const pcl::search::Search<pcl::PointXYZ>::Ptr& tree);

  void createMatrixFromFPFHS(
    pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr fpfhs, 
    Eigen::MatrixXf& output);
} // pcl

#endif // TOOLS_FEATURES_NORMALS_H
