#ifndef ENHANCED_SUPERVOXEL_CLUSTERING_H_
#define ENHANCED_SUPERVOXEL_CLUSTERING_H_

#include <map>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>
#include <pcl/search/kdtree.h>

class Segment;

namespace pcl {
  class EnhancedSupervoxelClustering : public pcl::PCLBase<pcl::PointXYZ> {
   protected:
    typedef pcl::PointXYZ PointT;
    using PCLBase <PointT>::initCompute;
    using PCLBase <PointT>::deinitCompute;
    using PCLBase <PointT>::input_;

    typedef pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> OctreeCloudSearchT;

    public:
    EnhancedSupervoxelClustering();

    void setSpatialImportance(float spatial_importance);
    void setNormalImportance(float normal_importance);
    void setFPFHImportance(float fpfh_importance);

    void setInputFPFHs(pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr fpfhs);
    void setInputNormals(pcl::PointCloud<pcl::Normal>::ConstPtr normals);

    void setKSearch(int k_search) {
        k_search_ = k_search;
    }

    void setRadiusSearch(float radius_search) {
        radius_search_ = radius_search;
    }

    void prepareVoxels(float voxel_resolution);
    void prepareSeeds(int nb_desired_clusters);
    void prepareSeeds(float seed_resolution);

    void computeVoxelMeans();
    void computeClusterMeans();
    float computeVoxelToClusterDistance(
        const Eigen::RowVectorXf& voxel_mean, 
        const Eigen::RowVectorXf& cluster_mean, 
        bool update_normalizers = false);

    void enforceConnectivity();
    void runKmeans(
        float max_cluster_spatial_distance, 
        float max_cluster_descr_distance);

    void cluster(
      float voxel_resolution, int nb_desired_clusters,
        std::vector<Segment>& segments,
        bool enforce_connectivity);

   protected:
    float fpfh_importance_, spatial_importance_, normal_importance_;
    float fpfh_dist_normalizer_, spatial_dist_normalizer_;
    float next_fpfh_dist_normalizer_, next_spatial_dist_normalizer_;
    pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr fpfhs_;
    pcl::PointCloud<pcl::Normal>::ConstPtr normals_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_centroids_;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr voxels_tree_;

    int num_voxels_, num_clusters_, k_search_;
    float radius_search_;
    std::vector<int> cluster_density_;
    Eigen::MatrixXf voxel_means_, cluster_means_;
    std::vector<int> point_voxels_, voxel_clusters_;
  };
}

#endif // ENHANCED_SUPERVOXEL_CLUSTERING_H_