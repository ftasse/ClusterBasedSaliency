#include "segmentation/enhanced_supervoxel_clustering.h"

#include <float.h>
#include <flann/algorithms/dist.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include "segmentation/segment.h"
#include "segmentation/segmentation_utils.h"
#include "tools/utils/pcl_utils.h"

namespace pcl {

  EnhancedSupervoxelClustering::EnhancedSupervoxelClustering():
    fpfh_importance_(1.0), spatial_importance_(1.0), normal_importance_(1.0), k_search_(0), radius_search_(0.03f) {
  }

  void EnhancedSupervoxelClustering::setSpatialImportance(float spatial_importance) {
    spatial_importance_ = spatial_importance;
  }

  void EnhancedSupervoxelClustering::setNormalImportance(float normal_importance) {
    normal_importance_ = normal_importance;
  }

  void EnhancedSupervoxelClustering::setFPFHImportance(float fpfh_importance) {
    fpfh_importance_ = fpfh_importance;
  }

  void EnhancedSupervoxelClustering::setInputFPFHs(pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr fpfhs) {
    fpfhs_ = fpfhs;
  }

  void EnhancedSupervoxelClustering::setInputNormals(pcl::PointCloud<pcl::Normal>::ConstPtr normals) {
    normals_ = normals;
  }

  void EnhancedSupervoxelClustering::prepareVoxels(float voxel_resolution) {
    voxel_centroids_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    point_voxels_.clear(); 
    point_voxels_.resize(input_->size(), -1);

    if (voxel_resolution <= 0) {
      *voxel_centroids_ = *input_;
      num_voxels_ = voxel_centroids_->size(); 
      for (size_t i = 0; i < input_->size(); ++i) point_voxels_[i] = i;
    } else {
      pcl::VoxelGrid<pcl::PointXYZ> grid;
      grid.setInputCloud (input_);
      float leaf_size = voxel_resolution;
      grid.setLeafSize (leaf_size, leaf_size, leaf_size);
      grid.setSaveLeafLayout(true);
      grid.filter (*voxel_centroids_);
      num_voxels_ = voxel_centroids_->size();
      if (num_voxels_ == input_->size()) {
        for (size_t i = 0; i < input_->size(); ++i) point_voxels_[i] = i;
      } else {
        for (size_t i = 0; i < input_->size(); ++i) {
          int voxel_id = grid.getCentroidIndex (input_->points[i]);
          point_voxels_[i] = voxel_id;
        }
      }

      /*OctreeCloudSearchT::Ptr octree_;    
      octree_.reset(new OctreeCloudSearchT(voxel_resolution));
      octree_->setInputCloud (input_);
      octree_->addPointsFromInputCloud ();
      num_voxels_ = 
          octree_->getOccupiedVoxelCenters(voxel_centroids_->points); 
      point_voxels_.clear(); 
      point_voxels_.resize(input_->size(), -1);
      for (size_t j = 0; j < voxel_centroids_->size(); ++j) {
        std::vector<int> points_indices_in_voxel;
        if (octree_->voxelSearch (
            voxel_centroids_->points[j], points_indices_in_voxel)) {
          for (size_t k = 0; k < points_indices_in_voxel.size(); ++k)
            point_voxels_[points_indices_in_voxel[k]] = j;
        }
      }*/
    }

    computeVoxelMeans();
    voxels_tree_.reset(new pcl::search::KdTree<pcl::PointXYZ> ());
    voxels_tree_->setInputCloud (voxel_centroids_);
    std::cout << "Num of voxels: " << voxel_resolution
              << " " << voxel_centroids_->size() << "\n";
  }

  void EnhancedSupervoxelClustering::prepareSeeds(float seed_resolution) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_centers_(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> grid;
    grid.setInputCloud (input_);
    float leaf_size = seed_resolution;
    grid.setLeafSize (leaf_size, leaf_size, leaf_size);
    grid.setSaveLeafLayout(true);
    grid.filter (*cluster_centers_);
    num_clusters_= cluster_centers_->size();
    voxel_clusters_.clear(); 
    voxel_clusters_.resize(voxel_centroids_->size(), -1);
    for (size_t j = 0; j < voxel_centroids_->size(); ++j) {
      int cluster_id = grid.getCentroidIndex (voxel_centroids_->points[j]);
      if (cluster_id >= 0 && cluster_id < num_clusters_)
        voxel_clusters_[j] = cluster_id;
    }
    for (size_t j = 0; j < voxel_centroids_->size(); ++j) {
      int cluster_id = voxel_clusters_[j];
      if (cluster_id < 0) {
        std::vector<int> k_neighbours(k_search_);
        std::vector<float> k_sqr_distances(k_search_);
        if (k_search_ > 0) voxels_tree_->nearestKSearch(j, k_search_, k_neighbours, k_sqr_distances);
        else voxels_tree_->radiusSearch(j, radius_search_, k_neighbours, k_sqr_distances);
        for (size_t k = 0; k < k_neighbours.size(); ++k)
          if (voxel_clusters_[k_neighbours[k]] >= 0) {
            voxel_clusters_[j] = voxel_clusters_[k_neighbours[k]];
            break;
          }
      }
    }
    
    /*float seed_resolution = voxel_resolution*sqrt(num_voxels_/nb_desired_clusters);
    OctreeCloudSearchT::Ptr sampling_octree_;
    sampling_octree_.reset(new OctreeCloudSearchT(seed_resolution));
    sampling_octree_->setInputCloud (voxel_centroids_);
    sampling_octree_->addPointsFromInputCloud ();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_centers_(
        new pcl::PointCloud<pcl::PointXYZ>);
    num_clusters_ = 
        sampling_octree_->getOccupiedVoxelCenters(cluster_centers_->points); 
    voxel_clusters_.clear(); 
    voxel_clusters_.resize(voxel_centroids_->size(), -1);
    for (size_t j = 0; j < cluster_centers_->size(); ++j) {
      std::vector<int> voxel_indices_in_cluster;
      if (sampling_octree_->voxelSearch (
          cluster_centers_->points[j], voxel_indices_in_cluster)) {
        for (size_t k = 0; k < voxel_indices_in_cluster.size(); ++k)
          voxel_clusters_[voxel_indices_in_cluster[k]] = j;
      }
    }*/
    computeClusterMeans();
    std::cout << "Num of clusters: " << seed_resolution
                << " " << num_clusters_ << "\n";
  }

  void EnhancedSupervoxelClustering::prepareSeeds(int nb_desired_clusters) {

    /*pcl::RandomSample<pcl::PointXYZ> sample (true); // Extract removed indices
    sample.setInputCloud (voxel_centroids_);
    sample.setSample (nb_desired_clusters);
    // sample.setSeed(0);
    std::vector<int> indices;
    sample.filter (indices);

    num_clusters_ = indices.size();
    voxel_clusters_.clear(); 
    voxel_clusters_.resize(voxel_centroids_->size(), -1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_centers_(
        new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t k = 0; k < indices.size(); ++k) {
      cluster_centers_->points.push_back(voxel_centroids_->points[indices[k]]);
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    tree->setInputCloud (cluster_centers_);
    std::vector<int> k_index(1);
    std::vector<float> k_sqr_distance(1);
    for (size_t j = 0; j < voxel_centroids_->size(); ++j) {
      if (tree->nearestKSearch(*voxel_centroids_, j, 1, k_index, k_sqr_distance)) {
        voxel_clusters_[j] = k_index[0];
      }
    }*/

    /*pcl::PointCloud<pcl::Normal>::Ptr voxel_normals(new pcl::PointCloud<pcl::Normal>);
    voxel_normals->resize(num_voxels_);
    for (size_t j = 0; j < num_voxels_; ++j) {
      pcl::Normal normal;
      normal.normal_x = normal.normal_y = normal.normal_z = normal.curvature = 0.0;
      voxel_normals->points[j] = normal;
    }
    std::vector<int> voxel_density(num_voxels_, 0);
    for (size_t i = 0; i < input_->size(); ++i) {
      int j = point_voxels_[i];
      if (j < 0)  continue;
      else voxel_density[j]++;
      voxel_normals->points[j].normal_x = normals_->points[i].normal_x;
      voxel_normals->points[j].normal_y = normals_->points[i].normal_y;
      voxel_normals->points[j].normal_z = normals_->points[i].normal_z;
      voxel_normals->points[j].curvature = normals_->points[i].curvature;
    }
    for (size_t j = 0; j < num_voxels_; ++j) {
      if (voxel_density[j] == 0)  continue;
      Eigen::Vector3f normal = voxel_normals->points[j].getNormalVector3fMap().normalized();
      voxel_normals->points[j].normal_x = normal[0];
      voxel_normals->points[j].normal_y = normal[1];
      voxel_normals->points[j].normal_z = normal[2];
      voxel_normals->points[j].curvature /= voxel_density[j];
    }*/
    std::vector<Segment> initial_segments;
    // computeClustersRegionGrowing(voxel_centroids_, voxel_normals, initial_segments, nb_desired_clusters, 9);
    classical_segment(voxel_centroids_, voxels_tree_, initial_segments, voxel_centroids_->size()/nb_desired_clusters, k_search_, radius_search_);
    num_clusters_ = initial_segments.size();
    voxel_clusters_.clear();
    voxel_clusters_.resize(voxel_centroids_->size(), -1);
    for (size_t k = 0; k < initial_segments.size(); ++k) {
      for (size_t l = 0; l < initial_segments[k].assignments.size(); ++l)
        voxel_clusters_[initial_segments[k].assignments[l]] = k;
    }

    computeClusterMeans();
    std::cout << "Num of clusters: " << nb_desired_clusters
                << " " << num_clusters_ << "\n";
  }

  void EnhancedSupervoxelClustering::computeVoxelMeans()  {
    voxel_means_ = Eigen::MatrixXf::Zero(num_voxels_, 3+3+33);
    std::vector<int> voxel_density(num_voxels_, 0);

    #pragma omp parallel for shared (voxel_density)
    for (size_t i = 0; i < input_->size(); ++i) {
      int voxel_id = point_voxels_[i];
      if (voxel_id < 0) continue;
      else voxel_density[voxel_id] += 1;

      Eigen::Vector3f point = input_->points[i].getVector3fMap();
      for (int c = 0; c < 3; ++c) 
        voxel_means_.row(voxel_id)[c] += point[c]; 

      if (normals_.get() && normals_->size() == input_->size()) {
        Eigen::Vector3f normal(
          normals_->points[i].normal_x, 
          normals_->points[i].normal_y, 
          normals_->points[i].normal_z);
        for (int c = 0; c < 3; ++c)
          voxel_means_.row(voxel_id)[3 + c] += normal[c];
      }

      if (fpfhs_.get() && fpfhs_->size() == input_->size()) {
        for (int c = 0; c < 33; ++c)
          voxel_means_.row(voxel_id)[6 + c] += fpfhs_->points[i].histogram[c];
      }
    }
    for (size_t j = 0; j < num_voxels_; ++j) {
      if (voxel_density[j] > 0) voxel_means_.row(j) /= voxel_density[j];
      // use voxel centroids instead of centers
      Eigen::Vector3f centroid = voxel_centroids_->points[j].getVector3fMap();
      voxel_means_.row(j).segment(0, 3) = centroid;
      // normalize normals
      Eigen::Vector3f n  = voxel_means_.row(j).segment(3, 3);
      if (n.squaredNorm() > 0) n = n.normalized();
      voxel_means_.row(j).segment(3, 3) = n;
    }
  }

  void EnhancedSupervoxelClustering::computeClusterMeans() {
    cluster_means_ = Eigen::MatrixXf::Zero(num_clusters_, voxel_means_.cols());
    cluster_density_ = std::vector<int>(num_clusters_, 0);
    
    #pragma omp parallel for
    for (size_t j = 0; j < num_voxels_; ++j) {
      int cluster_id = voxel_clusters_[j];
      if (cluster_id < 0) continue;
      cluster_density_[cluster_id] += 1;
      cluster_means_.row(cluster_id) += voxel_means_.row(j);
    }
    for (size_t k = 0; k < num_clusters_; ++k) {
      if (cluster_density_[k] > 0) cluster_means_.row(k) /= cluster_density_[k];
      Eigen::Vector3f n  = cluster_means_.row(k).segment(3, 3);
      if (n.squaredNorm() > 0) n = n.normalized();
      cluster_means_.row(k).segment(3, 3) = n;
    }
  }

  float EnhancedSupervoxelClustering::computeVoxelToClusterDistance(
      const Eigen::RowVectorXf& voxel_mean, 
      const Eigen::RowVectorXf& cluster_mean, 
      bool update_normalizers) {
    float spatial_dist = 
      (voxel_mean.segment(0, 3) - cluster_mean.segment(0, 3)).squaredNorm();
    Eigen::RowVectorXf v1 = voxel_mean.segment(6, 33);
    Eigen::RowVectorXf v2 = cluster_mean.segment(6, 33);
    float fpfh_dist = 
      flann::ChiSquareDistance<float>()(v1.data(), v2.data(), v1.size());

    if (update_normalizers) {
      next_spatial_dist_normalizer_ = 
          std::max(next_spatial_dist_normalizer_, spatial_dist);
      next_fpfh_dist_normalizer_ = 
          std::max(next_fpfh_dist_normalizer_, fpfh_dist);
    }

    float normal_angle_cosine = 0.0;
    if (normal_importance_*normal_importance_ > 0)
      1.0f - std::abs(voxel_mean.segment(3, 3).dot(cluster_mean.segment(3, 3)));
    return spatial_importance_*spatial_dist/spatial_dist_normalizer_ + 
           normal_importance_*normal_angle_cosine +
           fpfh_importance_*fpfh_dist/fpfh_dist_normalizer_;
  }

  void EnhancedSupervoxelClustering::enforceConnectivity() {
    for (size_t j = 0; j < num_voxels_; ++j) {
      int cluster_id = voxel_clusters_[j];
      int nearestK = 9;
      if (cluster_id == -1) nearestK = 2*cluster_density_[cluster_id];
      std::vector<int> neighbours(nearestK);
      std::vector<float> sqr_dists(nearestK);
      if (voxels_tree_->nearestKSearch(
        voxel_centroids_->points[j], nearestK, neighbours, sqr_dists)) {
        size_t nb_neighbours = std::min((size_t) 9, neighbours.size());
        int nb_similar_neighbours = 0;
        for (size_t c = 0; c < nb_neighbours ; ++c) {
          if (voxel_clusters_[neighbours[c]] == cluster_id)
            ++nb_similar_neighbours;
        }
        if (cluster_id < 0 || 
            nb_similar_neighbours < nb_neighbours/3) {
          int nearby_cluster_id = -1;
          for (size_t c = 0; c < neighbours.size(); ++c) {
            if (cluster_id >= 0 &&
                voxel_clusters_[neighbours[c]] != cluster_id) {
              nearby_cluster_id = voxel_clusters_[neighbours[c]]; break;
            }
          }
          if (nearby_cluster_id >= 0)
            voxel_clusters_[j] = nearby_cluster_id;
        }
      }
      
    }
    computeClusterMeans();
  }

  void EnhancedSupervoxelClustering::runKmeans(
    float max_cluster_spatial_distance, float max_cluster_descr_distance) {
    spatial_dist_normalizer_ = max_cluster_spatial_distance;
    fpfh_dist_normalizer_ = max_cluster_descr_distance;
    
    // Kmeans
    int max_iters = 30;
    int nb_iters = 0;
    float epsilon = 1e-4;
    float step = 2*sqrt(max_cluster_spatial_distance);
    int nb_neighbouring_clusters = 9;
    float prev_error = 0;
    Eigen::MatrixXf prev_cluster_means;
    std::vector<float> voxel_min_dist(num_voxels_, FLT_MAX);

    std::cout << "Step: " << step <<  "\n";

    while (nb_iters < max_iters) {
      ++ nb_iters;
      next_spatial_dist_normalizer_ = 0;
      next_fpfh_dist_normalizer_ = 0;

      // #pragma omp parallel for shared(voxel_min_dist)
      #pragma omp parallel for default(shared)
      for (size_t k = 0; k < num_clusters_; ++k) {
        if (cluster_density_[k] == 0) {
          // std::cout << k << ": empty cluster.\n"; 
          continue;
        }

        pcl::PointXYZ center;
        center.x = cluster_means_.row(k)[0];
        center.y = cluster_means_.row(k)[1];
        center.z = cluster_means_.row(k)[2];

        std::vector<int> neighbouring_voxels; 
        std::vector<float> sqr_dist;
        voxels_tree_->radiusSearch(
          center, step, neighbouring_voxels, sqr_dist);
        for (size_t c=0; c < neighbouring_voxels.size(); ++c) {
          int j = neighbouring_voxels[c];
          float dist =  computeVoxelToClusterDistance(
              voxel_means_.row(j), cluster_means_.row(k), 
              (voxel_clusters_[j] == k));
          if (dist < voxel_min_dist[j] - FLT_MIN) {
            voxel_clusters_[j] = k;
            voxel_min_dist[j] = dist;
          }
        }
      }

      spatial_dist_normalizer_ = next_spatial_dist_normalizer_;
      fpfh_dist_normalizer_ = next_fpfh_dist_normalizer_;

      prev_cluster_means = cluster_means_;
      computeClusterMeans();
      float error = 
        (prev_cluster_means.leftCols(3) -
         cluster_means_.leftCols(3)).cwiseAbs().sum();
      std::cout << "Iter " << nb_iters << ": " << error
                << " | normalizers: " << spatial_dist_normalizer_ << " "
                                      << fpfh_dist_normalizer_ << "\n";
      if (error < epsilon || std::abs(error-prev_error) < FLT_MIN)  break;
      prev_error = error;
    }
  }

  void EnhancedSupervoxelClustering::cluster(
    float voxel_resolution, int nb_desired_clusters,
      std::vector<Segment>& segments,
      bool enforce_connectivity) {
    float max_cluster_spatial_distance, max_cluster_descr_distance;
    voxel_resolution = std::max(voxel_resolution, 0.0001f);

    initCompute();
    prepareVoxels(voxel_resolution);
    float seed_resolution;
    if (num_voxels_ > 2*nb_desired_clusters)  {
      int voxels_per_seed = voxel_centroids_->size()/nb_desired_clusters;
      std::cout << "voxels_per_seed: " << voxels_per_seed << "\n";
      int increment = num_voxels_ / 100;
      std::vector<float> avg_distances, avg_xdist, avg_ydist, avg_zdist;
      for (int k = 0; k < voxel_centroids_->size(); k += increment) {
        std::vector<float> neighbours_sqr_dists; std::vector<int> neighbours_ids;
        voxels_tree_->nearestKSearch(k, voxels_per_seed, neighbours_ids, neighbours_sqr_dists);
        avg_distances.push_back(neighbours_sqr_dists.back()); 
        avg_xdist.push_back(2*std::abs(voxel_centroids_->points[k].x - voxel_centroids_->points[neighbours_ids.back()].x)); 
        avg_ydist.push_back(2*std::abs(voxel_centroids_->points[k].y - voxel_centroids_->points[neighbours_ids.back()].y));  
        avg_zdist.push_back(2*std::abs(voxel_centroids_->points[k].z - voxel_centroids_->points[neighbours_ids.back()].z)); 
      }
      std::sort(avg_distances.begin(), avg_distances.end());
      std::sort(avg_xdist.begin(), avg_xdist.end());
      std::sort(avg_ydist.begin(), avg_ydist.end());
      std::sort(avg_zdist.begin(), avg_zdist.end());
      float radius = sqrt(avg_distances[avg_distances.size()/2]);
      float avg_voxel_length = std::max(std::max(avg_zdist[avg_xdist.size()/2], avg_zdist[avg_ydist.size()/2]), avg_zdist[avg_zdist.size()/2]);
      seed_resolution = 2*radius; // for a cube with side "x", the length of the diagonal is d=x*sqrt(3) 
      // seed_resolution = sqrt(3)*avg_voxel_length;
      std::cout << "seed radius: " << radius << " " <<  avg_voxel_length << "\n";
      // seed_resolution *= pow(voxel_centroids_->size()/nb_desired_clusters, 1.0/2.0);
    } else {
      seed_resolution = pcl::computeAverageSpacing(voxel_centroids_, 9, voxels_tree_);
    }
    prepareSeeds(seed_resolution);

    if (fpfh_importance_ >= 0) {
      max_cluster_spatial_distance = 0.0;
      max_cluster_descr_distance = 0.0;
      for (size_t j = 0; j < voxel_centroids_->size(); ++j) {
        int cluster_id = voxel_clusters_[j];
        if (cluster_id >= 0) { 
          float spatial_dist = (voxel_means_.row(j).leftCols(3) - 
                                cluster_means_.row(cluster_id).leftCols(3)).squaredNorm();
          Eigen::RowVectorXf voxel_descr = voxel_means_.row(j).segment(6, 33);
          Eigen::RowVectorXf cluster_descr = cluster_means_.row(cluster_id).segment(6, 33);
          float descr_dist = 
              flann::ChiSquareDistance<float>()(
                  voxel_descr.data(), cluster_descr.data(), voxel_descr.size());
          max_cluster_descr_distance = std::max(max_cluster_descr_distance, descr_dist);
          max_cluster_spatial_distance = std::max(max_cluster_spatial_distance, spatial_dist);
        }
      }

      runKmeans(max_cluster_spatial_distance, max_cluster_descr_distance);
      if (enforce_connectivity) enforceConnectivity();
    }

    segments.clear(); 
    segments.reserve(num_clusters_);
    std::map<int, int> cluster_to_segment;
    for (size_t k = 0; k < num_clusters_; ++k)
      if (cluster_density_[k] > 0) {
        segments.push_back(Segment());
        cluster_to_segment[k] = segments.size()-1;
      }
    for (size_t i = 0; i < input_->size(); ++i) {
      int voxel_id = point_voxels_[i];
      if (voxel_id < 0) {
        std::cout << "Point with no voxel label: " << i << "\n";
        continue;
      }
      int cluster_id = voxel_clusters_[voxel_id];
      if (cluster_id < 0) {
        std::cout << "Voxel with no label: " << voxel_id << "\n";
        continue;
      }
      segments[cluster_to_segment[cluster_id]].assignments.push_back(i);
    }
  }
}