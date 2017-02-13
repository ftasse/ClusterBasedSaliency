#include "utils/pcl_utils.h"

#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>

namespace pcl {
  ModelSampler::ModelSampler(pcl::PolygonMesh& mesh, const std::vector<float>& scalar, bool use_topology) :
    mesh_(&mesh) {

    point_normals_.reset(new pcl::PointCloud<pcl::PointNormal>);
    pcl::fromPCLPointCloud2(mesh_->cloud, *point_normals_);

    if (use_topology && mesh_->polygons.size() > 0) {
      std::vector<float>  face_areas (mesh_->polygons.size(), 0.0);
      std::vector<float>  face_scalar (mesh_->polygons.size(), 0.0);
      for (size_t f = 0; f  < face_areas.size(); f++) {
        if (mesh_->polygons[f].vertices.size() < 3) continue;
        pcl::PointNormal a = point_normals_->points[mesh_->polygons[f].vertices[0]];
        pcl::PointNormal b = point_normals_->points[mesh_->polygons[f].vertices[1]];
        pcl::PointNormal c = point_normals_->points[mesh_->polygons[f].vertices[2]];
        Eigen::Vector3f v1 = b.getVector3fMap() - a.getVector3fMap();
        Eigen::Vector3f v2 = c.getVector3fMap() - a.getVector3fMap(); 
        face_areas[f] = 0.5*v1.cross(v2).norm();
        if (scalar.size() == point_normals_->size()) 
          face_scalar[f] = (scalar[mesh_->polygons[f].vertices[0]] +
            scalar[mesh_->polygons[f].vertices[1]] + scalar[mesh_->polygons[f].vertices[2]])/3.0;
      }
      face_generator_ = std::default_random_engine(time(NULL));
      if (scalar.size() != point_normals_->size()) 
        face_distribution_ = std::discrete_distribution<int>(
          face_areas.begin(), face_areas.end());
      else
        face_distribution_ = std::discrete_distribution<int>(
          face_scalar.begin(), face_scalar.end());
    } else {
      point_generator_ = std::default_random_engine(time(NULL));
      if (scalar.size() == point_normals_->size())
        point_distribution_ = std::discrete_distribution<int>(
          scalar.begin(), scalar.end());
      else {
        std::vector<float> uniform(point_normals_->size(), 1.0/point_normals_->size());
        point_distribution_ = std::discrete_distribution<int>(
          uniform.begin(), uniform.end());
      }
    }
  }

  PointNormal ModelSampler::getRandomPoint() {
    if (face_distribution_.probabilities().size() > 0 && mesh_->polygons.size() > 0) {
      int rand_polygon_id = -1;
      do  {
        rand_polygon_id = face_distribution_(face_generator_);
      } while (mesh_->polygons[rand_polygon_id].vertices.size() < 3);
      pcl::PointNormal a = point_normals_->points[mesh_->polygons[rand_polygon_id].vertices[0]];
      pcl::PointNormal b = point_normals_->points[mesh_->polygons[rand_polygon_id].vertices[1]];
      pcl::PointNormal c = point_normals_->points[mesh_->polygons[rand_polygon_id].vertices[2]];
      float ra = rand() / (float) RAND_MAX;
      float rb = rand() / (float) RAND_MAX;
      float rc = rand() / (float) RAND_MAX;
      pcl::PointNormal new_point;
      new_point.x = (ra*a.x + rb*b.x + rc*c.x)/(ra+rb+rc);
      new_point.y = (ra*a.y + rb*b.y + rc*c.y)/(ra+rb+rc);
      new_point.z = (ra*a.z + rb*b.z + rc*c.z)/(ra+rb+rc);
      new_point.normal_x = (ra*a.normal_x + rb*b.normal_x + rc*c.normal_x)/(ra+rb+rc);
      new_point.normal_y = (ra*a.normal_y + rb*b.normal_y + rc*c.normal_y)/(ra+rb+rc);
      new_point.normal_z = (ra*a.normal_z + rb*b.normal_z + rc*c.normal_z)/(ra+rb+rc);
      new_point.curvature = (ra*a.curvature + rb*b.curvature + rc*c.curvature)/(ra+rb+rc);
      return new_point;
    } else {
      int rand_point_id = point_distribution_(point_generator_);
      return point_normals_->points[rand_point_id];
    }
  }

  template <typename T>
  void applyLloydRelaxation(
   typename pcl::PointCloud<T>::Ptr& all_points,
   typename pcl::PointCloud<T>::Ptr& input_points) {

    typename pcl::search::KdTree<T>::Ptr tree (new pcl::search::KdTree<T>);
    tree->setInputCloud (input_points);

    float eps = 1e-6;
    float prev_mse = 0;
    int n_iters = 0;
    int max_iters = 30;

    while (true) {
      n_iters++;
      std::vector<int> num_of_closest_to_point(input_points->size(), 0);
      std::vector<Eigen::Vector3f> new_centroids(input_points->size(), Eigen::Vector3f(0,0,0));

      std::vector<int> nearest_id(1);
      std::vector<float> nearest_dist(1);
      for (int i = 0; i < all_points->size(); ++i) {
        tree->nearestKSearch(all_points->points[i], 1, nearest_id, nearest_dist);
        if (nearest_id.size() == 1) {
          int j = nearest_id[0];
          num_of_closest_to_point[j]++;
          new_centroids[j] += all_points->points[i].getVector3fMap();
        }
      }

      std::vector<int> to_delete;
      float mse = 0;

      for (int j = 0; j < input_points->size(); ++j) {
        if (num_of_closest_to_point[j] == 0) {
          to_delete.push_back(j);
        } else {
          new_centroids[j] /= num_of_closest_to_point[j];
          mse += (input_points->points[j].getVector3fMap() - new_centroids[j]).squaredNorm();
          input_points->points[j].getVector3fMap() = new_centroids[j];
        }
      }

      for (int k = ((int) to_delete.size())-1; k >= 0; --k) {
        int j = to_delete[k];
        input_points->points.erase(input_points->points.begin() + j);
      }

      std::cout << "iter " << n_iters << ": " << prev_mse << " " << mse << "\n";
      if (std::abs(mse - prev_mse) < eps || n_iters == max_iters) break;
      
      prev_mse = mse;
      tree->setInputCloud (input_points);
    }

  }


  float computeAverageSpacing(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr points, 
        size_t num_neighbours,
        pcl::search::Search<pcl::PointXYZ>::ConstPtr tree) {
   float mean = 0.0;
   unsigned long counter = 0;
    std::vector<int> pointIdxRadiusSearch(num_neighbours);
    std::vector<float> pointRadiusSquaredDistance(num_neighbours);  

    #pragma omp parallel for default(shared) private(pointIdxRadiusSearch, pointRadiusSquaredDistance)  num_threads(0)
    for (size_t i = 0; i < points->size(); ++i) {
      if ( tree->nearestKSearch (points->points[i], num_neighbours, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ) {
        if (pointIdxRadiusSearch.size() > 1) {
          mean += Eigen::Map<Eigen::VectorXf>(
              &pointRadiusSquaredDistance[1], 
              pointRadiusSquaredDistance.size()-1).mean();
          counter += 1;
          // if (counter % (points->size()/5) == 0) std::cout << "Progress ... " << counter << "\n";
        }
      }
    }
    // std::cout << "\n";
    if (counter > 0) mean /= counter;
    return sqrt(mean);
  }

  int computeAverageDensity(
      pcl::PointCloud<pcl::PointXYZ>::ConstPtr points, 
      float radius,
      pcl::search::Search<pcl::PointXYZ>::ConstPtr tree) {
    unsigned long mean = 0;
    unsigned long counter = 0;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance; 
    #pragma omp parallel for default(shared) private(pointIdxRadiusSearch, pointRadiusSquaredDistance)  num_threads(0)
    for (size_t i = 0; i < points->size(); ++i) {
      tree->radiusSearch (points->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
      mean += (pointIdxRadiusSearch.size()-1);
      counter += 1;
      // if (counter % (points->size()/5) == 0) std::cout << "Progress ... " << counter << "\n";
    }
    // std::cout << "\n";
    if (counter > 0) mean /= counter;
    return mean;
  }

  /*void computeNeighbours(pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
			 std::vector<std::vector<size_t> >& neighbours,
			 std::vector<std::vector<float> >& neighbours_sqrdist,
			 float support_radius) {
    neighbours.clear();
    neighbours_sqrdist.clear();
    neighbours.resize(points->points.size());
    neighbours_sqrdist.resize(neighbours.size());
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (points);

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    for (size_t i = 0; i < neighbours.size(); ++i)
	if ( tree->radiusSearch (points->points[i], support_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
	    for (size_t k = 0; k < pointIdxRadiusSearch.size (); ++k) {
		neighbours[i].push_back(pointIdxRadiusSearch[k]);
		neighbours_sqrdist[i].push_back(pointRadiusSquaredDistance[k]);
	      }
  }

  void computeNeighbours(pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
			  std::vector<std::vector<size_t> > &neighbours,
			  std::vector<std::vector<float> > &neighbours_sqrdist,
			  int nb_neighbours) {
     neighbours.clear();
     neighbours_sqrdist.clear();
     neighbours.resize(points->points.size());
     neighbours_sqrdist.resize(neighbours.size());

     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
     tree->setInputCloud (points);

     for (size_t i = 0; i < neighbours.size(); ++i) {
        std::vector<int> pointIdxRadiusSearch(nb_neighbours);
        std::vector<float> pointRadiusSquaredDistance(nb_neighbours);
     
    	 if ( tree->nearestKSearch (points->points[i], nb_neighbours, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
    	     for (size_t k = 0; k < pointIdxRadiusSearch.size (); ++k) {
        		 neighbours[i].push_back(pointIdxRadiusSearch[k]);
        		 neighbours_sqrdist[i].push_back(pointRadiusSquaredDistance[k]);
        	  }
      }
   }*/

  float getNormalizingScaleFactor(
        pcl::PointCloud<pcl::PointXYZ>::Ptr &xyz_cloud, bool update) {
    pcl::PointCloud<pcl::PointXYZ> tmp_cloud;

    Eigen::Vector4f centroid;
    float scalefactor = 1.0;

    pcl::compute3DCentroid (*xyz_cloud, centroid);
    pcl::demeanPointCloud (*xyz_cloud, centroid, tmp_cloud);

    float max_sqr_distance = 0, d;
    pcl::PointXYZ cog (0, 0, 0);

    for (size_t i = 0; i < tmp_cloud.points.size (); ++i)
    {
        d = pcl::squaredEuclideanDistance(cog, tmp_cloud.points[i]);
        if (d > max_sqr_distance) max_sqr_distance = d;
    }

    // std::cout << "rad_real: " << centroid.segment(0,3).transpose() << " | " << sqrt(max_sqr_distance) << "\n";

    float scale_factor = 1.0f / (sqrt(max_sqr_distance) * scalefactor);

    if (update)
    {
        Eigen::Affine3f matrix = Eigen::Affine3f::Identity();
        matrix.scale (scale_factor);
        pcl::transformPointCloud (tmp_cloud, *xyz_cloud, matrix);
    }
    return scale_factor;
  }


  template void applyLloydRelaxation <pcl::PointXYZ>(
    pcl::PointCloud<pcl::PointXYZ>::Ptr&, pcl::PointCloud<pcl::PointXYZ>::Ptr&);

  template void applyLloydRelaxation <pcl::PointNormal>(
    pcl::PointCloud<pcl::PointNormal>::Ptr&, pcl::PointCloud<pcl::PointNormal>::Ptr&);

} // pcl
