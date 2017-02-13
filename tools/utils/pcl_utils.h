#ifndef TOOLS_PCLUTILS_H
#define TOOLS_PCLUTILS_H

#include <vector>
#if __cplusplus > 199711L
#include <random>
#endif
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <pcl/search/search.h>

namespace pcl {
  class PointXYZ;

  class ModelSampler {
  public:
      ModelSampler(pcl::PolygonMesh& mesh, const std::vector<float>& scalar = std::vector<float>(), bool use_topology = true);
      PointNormal getRandomPoint(); 
   private:
      pcl::PolygonMesh* mesh_;
      pcl::PointCloud<pcl::PointNormal>::Ptr point_normals_;
      #if __cplusplus > 199711L
      std::default_random_engine face_generator_;
      std::discrete_distribution<int> face_distribution_;
      std::default_random_engine point_generator_;
      std::discrete_distribution<int> point_distribution_;
      #endif
  };

  template <typename T>
  void applyLloydRelaxation(
   typename pcl::PointCloud<T>::Ptr& all_points,
   typename pcl::PointCloud<T>::Ptr& input_points);

   float computeAverageSpacing(
      pcl::PointCloud<pcl::PointXYZ>::ConstPtr points, 
      size_t num_neighbours,
      pcl::search::Search<pcl::PointXYZ>::ConstPtr tree);
   
   int computeAverageDensity(
      pcl::PointCloud<pcl::PointXYZ>::ConstPtr points, 
      float radius,
      pcl::search::Search<pcl::PointXYZ>::ConstPtr tree);

   /*void computeNeighbours(pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
			  std::vector<std::vector<size_t> >& neighbours,
			  std::vector<std::vector<float> >& neighbours_sqrdist,
			  float support_radius);

   void computeNeighbours(pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
			  std::vector<std::vector<size_t> > &neighbours,
			  std::vector<std::vector<float> > &neighbours_sqrdist,
			  int nb_neighbours);*/

   float getNormalizingScaleFactor(
   	pcl::PointCloud<pcl::PointXYZ>::Ptr &xyz_cloud, bool update);

} // pcl

#endif // TOOLS_PCLUTILS_H
