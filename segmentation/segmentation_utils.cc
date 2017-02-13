#include "segmentation/segmentation_utils.h"

#include <float.h>
#include <set>
#include <pcl/point_types.h>
#include <pcl/segmentation/region_growing.h>

#include "segmentation/enhanced_supervoxel_clustering.h"
#include "segmentation/segment.h"
#include "tools/utils/pcl_utils.h"

namespace pcl {

  void computeSuperClustersPapon(
      pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
      pcl::PointCloud<pcl::Normal>::ConstPtr normals,
      pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr fpfhs,
      std::vector<Segment>& segments, size_t nb_desired_clusters,
      float voxel_resolution, int nb_neighbours, float neighbourhood_radius, bool enforce_connectivity,
      float spatial_importance, float normal_importance, float fpfh_importance) {
    EnhancedSupervoxelClustering super_clusterer;
    super_clusterer.setInputCloud (points);
    super_clusterer.setInputNormals (normals);
    super_clusterer.setInputFPFHs (fpfhs);
    super_clusterer.setSpatialImportance (spatial_importance);
    super_clusterer.setNormalImportance (normal_importance);
    super_clusterer.setFPFHImportance (fpfh_importance);
    super_clusterer.setKSearch(nb_neighbours);
    super_clusterer.setRadiusSearch(neighbourhood_radius);
    super_clusterer.cluster(
      voxel_resolution, nb_desired_clusters, segments, enforce_connectivity);
  }

void computeSegmentAdjacencyWithNeighbourhood(
    const std::vector<Segment>& segments, 
    const pcl::search::Search<pcl::PointXYZ>::Ptr& tree,
    int nb_neighbours, float neighbourhood_radius,
    std::vector< std::vector<int> >& segment_adjacency) {
  int num_points = tree->getInputCloud()->size();
  std::vector<int> labels(num_points, -1);
  for (size_t j = 0; j < segments.size(); ++j)
    for (size_t k = 0; k < segments[j].num_points(); ++k)
      labels[segments[j].assignments[k]] = j;

  std::vector<int> nn_indices(nb_neighbours);
  std::vector<float> nn_dists(nb_neighbours);

  std::vector< std::set<int> > tmp_segment_adjacency(segments.size());
  for (size_t j = 0; j < segments.size(); ++j) {
  	for (size_t k = 0; k < segments[j].assignments.size(); ++k) {
  	    int idx = segments[j].assignments[k];
        if (nb_neighbours >0) tree->nearestKSearch(idx, nb_neighbours, nn_indices, nn_dists);
        else tree->radiusSearch(idx, neighbourhood_radius, nn_indices, nn_dists);
        if (nn_indices.size() <= 1) tree->nearestKSearch(idx, 9, nn_indices, nn_dists);
        for (int n = 0; n < nn_indices.size(); ++n) {
          int neigh_segment = labels[nn_indices[n]];
		      if (neigh_segment >= 0 && neigh_segment != j) {
		        tmp_segment_adjacency[j].insert(neigh_segment);
            //tmp_segment_adjacency[neigh_segment].insert(j);
          }
  		  }
  	 }
  }

  segment_adjacency.resize(segments.size());
  for (size_t  j = 0; j < segments.size(); ++j) {
    segment_adjacency[j]  = std::vector<int>(tmp_segment_adjacency[j].begin(),
                                             tmp_segment_adjacency[j].end());
  }
}

void computeClustersRegionGrowing(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& points,
      const pcl::PointCloud<pcl::Normal>::Ptr& normals,
      const pcl::search::Search<pcl::PointXYZ>::Ptr& tree,
      std::vector<Segment>& segments, 
      size_t nb_desired_clusters, size_t nb_neighbours) {
  float avg_density = points->size()/nb_desired_clusters;

  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  // reg.setMinClusterSize (avg_density*0.5);
  // reg.setMaxClusterSize (avg_density*1.5);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (nb_neighbours);
  reg.setInputCloud (points);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (5.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (1.0);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);
  segments.clear();
  for (size_t j = 0; j < clusters.size(); ++j) {
    segments.push_back(Segment());
    segments.back().assignments = clusters[j].indices;
  }
  std::cout << "Num of clusters from region growing: " << clusters.size() << "\n";
}

void classical_segment(
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_points,
  const pcl::search::Search<pcl::PointXYZ>::Ptr& tree,
  std::vector<Segment> &segments,
  int size_thres,
  int nb_neighbours, float neighbourhood_radius
) {
  float spatial_var = 0.0;
  Eigen::MatrixXf points = cloud_points->getMatrixXfMap(3, 4, 0).transpose();
  Eigen::RowVectorXf global_spatial_centroid = points.colwise().mean();

  for (size_t idx = 0; idx < points.rows(); ++idx)
  {
    spatial_var += (points.row(idx) - global_spatial_centroid).squaredNorm();
  }
  spatial_var /= points.rows();

  if (size_thres == 0) size_thres = std::max(2, (int)(points.rows() / 10000));
  float resolution = 0.0;
  //if (resolution < FLT_MIN)  resolution = spatial_var*0.0004f;

  std::set<size_t> available;
  for (size_t i = 0; i < points.rows(); ++i) available.insert(i);

  std::set<size_t> border;

  segments.clear();
  std::vector<int> cur_seg, cur_seed;
  border.insert(*available.begin());

  while (!border.empty()) {
    cur_seg.clear();
    cur_seed.clear();

    size_t cur_point = *border.begin();

    cur_seg.push_back(cur_point);
    cur_seed.push_back(cur_point);
    available.erase(cur_point);
    border.erase(cur_point);

    Eigen::RowVectorXf centroid = points.row(cur_point);

    for (size_t i = 0; i < cur_seed.size(); ++i)
    {
      size_t seed_pt_id = cur_seed[i];
      //Eigen::VectorXi seed_pt_neighbours = neighbours.row(seed_pt_id);
      std::vector<float> sqr_distances(nb_neighbours);
      std::vector<int> seed_pt_neighbours(nb_neighbours);
      if (nb_neighbours > 0) tree->nearestKSearch(seed_pt_id, nb_neighbours, seed_pt_neighbours, sqr_distances);
      else tree->radiusSearch(seed_pt_id, neighbourhood_radius, seed_pt_neighbours, sqr_distances);

      for (size_t j = 0; j< seed_pt_neighbours.size(); ++j)
        if (seed_pt_neighbours[j] < 0) break;
        else
        {
          size_t neigh_pt_id = seed_pt_neighbours[j];
          
          std::set<size_t>::iterator neigh_pt_pos = available.find(neigh_pt_id);
          if  (neigh_pt_pos == available.end()) continue;

          bool success = ((cur_seg.size()+1) < size_thres);
          //bool success = (points.row(cur_point)  - points.row(neigh_pt_id)).squaredNorm() < resolution;
          //bool success = (((centroid*(cur_seg.size()) + points.row(neigh_pt_id))/(cur_seg.size()+1))  - points.row(neigh_pt_id)).squaredNorm() < resolution;

          if (success)    {
            centroid = ((centroid*(cur_seg.size()) + points.row(neigh_pt_id))/(cur_seg.size()+1));
            cur_seg.push_back(neigh_pt_id);
            available.erase(neigh_pt_pos);
            cur_seed.push_back(neigh_pt_id);

            //std::set<size_t>::iterator border_pos = border.find(neigh_pt_id);
            //if (border_pos != border.end()) border.erase(border_pos);
          } //else  border.insert(neigh_pt_id);
        }
    }

    if (border.empty() && !available.empty()) border.insert(*available.begin());

    segments.push_back(Segment());
    segments.back().assignments = cur_seg;
  }

  std::cout << "Num of points: " << segments.size()<< " " << points.rows() << " " << size_thres << " " << resolution << " " << "\n";
  // std::cout << segments_[0].assignments[10] << " " << segments_[1].assignments[10] << "\n";
  /*if (cleanup){
    cleanupClusters(size_thres_);
    std::cout << "Num of points: " << segments_.size() << "   " << "\n\n";
  }*/
}

void cleanup_segment(
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_points,
  const pcl::search::Search<pcl::PointXYZ>::Ptr& tree,
  int nb_neighbours, float neighbourhood_radius,
  std::vector<Segment> &segments,
  int size_thres
) {
  Eigen::MatrixXf points = cloud_points->getMatrixXfMap(3, 4, 0).transpose();
  int num_points = points.rows();
  std::vector<int> labels(num_points, -1);
  for (size_t j = 0; j < segments.size(); ++j)
    for (size_t k = 0; k < segments[j].num_points(); ++k)
      labels[segments[j].assignments[k]] = j;

  if (size_thres == 0)
  {
    size_thres = (num_points/segments.size())*0.3;
    std::cout << "Size thres: " << size_thres << "\n";
  }


  std::vector<int> deleted_segments;

  std::vector<int> nn_indices(nb_neighbours);
  std::vector<float> nn_dists(nb_neighbours);

  for (size_t j = 0; j < segments.size(); ++j)
  {
    if (segments[j].num_points() < size_thres)
    {
      int closest_segment_j = -1;
      float min_dist = FLT_MAX;

      for (size_t k = 0; k < segments[j].num_points(); ++k)
      {
        int i = segments[j].assignments[k];
        if (nb_neighbours > 0) tree->nearestKSearch(i, nb_neighbours, nn_indices, nn_dists);
        else tree->radiusSearch(i, neighbourhood_radius, nn_indices, nn_dists);

        for (size_t l = 0; l < nn_indices.size(); ++l)
          if (nn_indices[l] >= 0 && labels[nn_indices[l]] != j)
          {
            if (closest_segment_j >= 0 && segments[labels[nn_indices[l]]].num_points() < size_thres)  
              continue;
            if (nn_dists[l] < min_dist)
            {
              closest_segment_j = labels[nn_indices[l]];
              min_dist = nn_dists[l];
            }
          }
      }

      if (closest_segment_j >= 0)
      {
        segments[closest_segment_j].assignments.insert(
          segments[closest_segment_j].assignments.end(), 
          segments[j].assignments.begin(), 
          segments[j].assignments.end());
        segments[j].assignments.clear();
        deleted_segments.push_back(j);
      }
    }
  }

  for (int k = ((int)deleted_segments.size())-1; k >= 0; --k)
  {
    int j = deleted_segments[k];
    segments.erase(segments.begin() + j);
  }

  for (size_t j = 0; j < segments.size(); ++j)
    if (segments[j].assignments.size() < size_thres)
      std::cout << j << " |---------| " << segments[j].assignments.size() << "\n";
}

void convertSegmentsToRGBs(const std::vector<Segment>& segments,
         pcl::PointCloud<pcl::RGB>::Ptr colors,
         size_t nb_points) {
    colors->points.clear();
    colors->width = nb_points;
    colors->height = 1;
    colors->points.resize(nb_points);
    
    for (size_t s = 0; s < segments.size(); ++s) {
      pcl::RGB color;
      color.r = rand()%255;
      color.b = rand()%255;
      color.g = rand()%255;
      for (size_t k = 0; k < segments[s].num_points(); ++k) {
        colors->points[segments[s].assignments[k]] = color;
      }
    }
  }

void convertSegmentsBoundariesToBlack(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
    const pcl::search::Search<pcl::PointXYZ>::Ptr& tree,
    const std::vector<Segment>& segments,
    pcl::PointCloud<pcl::RGB>::Ptr colors,
    size_t nb_points) {
   std::vector<int> point_labels(nb_points, -1);
   for (size_t s = 0; s < segments.size(); ++s) {
      for (size_t k = 0; k < segments[s].num_points(); ++k) {
        point_labels[segments[s].assignments[k]] = s;
      }
    }
    convertSegmentsBoundariesToBlack(points, tree, point_labels, colors, nb_points);
}

void convertSegmentsBoundariesToBlack(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
    const pcl::search::Search<pcl::PointXYZ>::Ptr& tree,
    const std::vector<int>& point_labels,
    pcl::PointCloud<pcl::RGB>::Ptr colors,
    size_t nb_points) {
  colors->width = nb_points;
  colors->height = 1;
  colors->points.resize(nb_points);

  pcl::RGB mesh_color, boundary_color;
  mesh_color.r = mesh_color.b = mesh_color.g = 255;
  boundary_color.r = boundary_color.b = boundary_color.g = 0;

  for (size_t i = 0; i < nb_points; ++i) colors->points[i] = mesh_color;

  int nb_neighbours = 6;
  std::vector<int> nn_indices(nb_neighbours);
  std::vector<float> nn_dists(nb_neighbours);
  
  for (size_t i = 0; i < nb_points; ++i) {
    if (!tree->nearestKSearch(i, nb_neighbours, nn_indices, nn_dists)) continue;
    for (size_t j = 0; j < nn_indices.size(); ++j) {
      if (point_labels[nn_indices[j]] != point_labels[i]) {
        colors->points[i] = boundary_color; break;
      }
    }
  }
}

}  // namespace pcl

