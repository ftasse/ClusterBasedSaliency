
#include "features/normals.h"

#include <float.h>
#include <fstream>
#include <pcl/for_each_type.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/pfh_tools.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_traits.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/Vertices.h>

#include <pcl/common/centroid.h>

#include "utils/eigen3_utils.h"
#include "utils/pcl_utils.h"
#include "utils/cgal_utils.h"

namespace pcl {

/*template <typename PointInT, typename PointNT, typename PointOutT>
class CustomFPFHEstimationOMP : public pcl::FPFHEstimationOMP<PointInT, PointNT, PointOutT> {
public:
  using Feature<PointInT, PointOutT>::feature_name_;

  CustomFPFHEstimationOMP (unsigned int nr_threads = 0) : 
      pcl::FPFHEstimationOMP<PointInT, PointNT, PointOutT>(nr_threads) {
    feature_name_ = "CustomFPFHEstimationOMP";
  }

  bool
  computePairFeatures (
      const pcl::PointCloud<PointInT> &cloud, const pcl::PointCloud<PointNT> &normals,
      int p_idx, int q_idx, float &f1, float &f2, float &f3, float &f4) {
    std::cout << "ran\n";
    if (!FPFHEstimationOMP<PointInT, PointNT, PointOutT>::computePairFeatures(
          cloud, normals, p_idx, q_idx, f1, f2, f3, f4)) return false;
    f1 = std::abs(f1);
    f2 = std::abs(f2);
    f3 = std::abs(f3);
    f4 = std::abs(f4);
    return true;
  }
};*/

template <typename PointT>
struct FieldMatcher {
  FieldMatcher (const std::vector<pcl::PCLPointField>& fields, bool* result)
    : fields_(fields), result_(result)  {
    *result_ = true;
  }

  template <typename Tag> void
  operator() () {
    bool has_field = false;
    for (int i = 0; i < fields_.size(); ++i) {
      if (pcl::FieldMatches<PointT, Tag>()(fields_[i])) {
	has_field = true; break;
      }
    }
    *result_ = *result_ && has_field;
    //printf("field: %d %d\n", has_field, result);
  }

  const std::vector<pcl::PCLPointField>& fields_;
  bool* result_;
};

bool cloudHasNormals(const pcl::PCLPointCloud2& cloud) {
  bool has_normals;
  FieldMatcher<pcl::Normal> field_matcher(cloud.fields, &has_normals);
  pcl::for_each_type<pcl::traits::fieldList<pcl::Normal>::type>(field_matcher);
  return has_normals;
}

void computeNormals(const std::vector<pcl::Vertices> &polygons,
	      pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
	      pcl::PointCloud<pcl::Normal>::Ptr normals) {
  std::vector<Eigen::Vector3f> vertex_normals(points->points.size(), Eigen::Vector3f(0, 0, 0));
  std::vector<Eigen::Vector3f> face_normals(polygons.size(), Eigen::Vector3f(0, 0, 0));

  #pragma omp parallel for shared (polygons, vertex_normals)  num_threads(0)
  for (size_t i = 0; i < polygons.size(); ++i)
    if (polygons[i].vertices.size() >= 3) {
    	size_t id0 = polygons[i].vertices[0];
    	size_t id1 = polygons[i].vertices[1];
    	size_t id2 = polygons[i].vertices[2];

    	pcl::PointXYZ p0 = points->points[id0];
    	pcl::PointXYZ p1 = points->points[id1];
    	pcl::PointXYZ p2 = points->points[id2];

    	Eigen::Vector3f vec1 (p1.x-p0.x, p1.y-p0.y, p1.z-p0.z);
    	Eigen::Vector3f vec2 (p2.x-p0.x, p2.y-p0.y, p2.z-p0.z);
    	face_normals[i] = vec1.cross(vec2);

    	// no normalization = weighting by surface!
    	// float norm = face_normals[i].norm();
    	// if (norm > FLT_MIN)face_normals[i] /= norm;

    	vertex_normals[id0] += face_normals[i];
    	vertex_normals[id1] += face_normals[i];
    	vertex_normals[id2] += face_normals[i];
    }

  normals->points.resize(vertex_normals.size());
  #pragma omp parallel for shared (normals, vertex_normals)  num_threads(0)
  for (size_t i = 0; i < normals->points.size(); ++i) {
    float norm = vertex_normals[i].norm();
    if (norm > FLT_MIN)vertex_normals[i] /= norm;
  	normals->points[i].normal[0] = vertex_normals[i][0];
  	normals->points[i].normal[1] = vertex_normals[i][1];
  	normals->points[i].normal[2] = vertex_normals[i][2];
  }
}

void computeNormals(pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
		    pcl::PointCloud<pcl::Normal>::Ptr normals,
		    float support_radius,
        const pcl::search::Search<pcl::PointXYZ>::Ptr& tree
) {
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (points);
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (support_radius);
  ne.compute (*normals);
}

void computeNormals(pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
        pcl::PointCloud<pcl::Normal>::Ptr normals,
        size_t nb_neighbours,
        const pcl::search::Search<pcl::PointXYZ>::Ptr& tree
) {
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (points);
  ne.setSearchMethod (tree);
  ne.setKSearch (nb_neighbours);
  ne.compute (*normals);
}

bool computeAndOrientNormalsWithCGAL(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
    pcl::PointCloud<pcl::Normal>::Ptr normals,
    size_t n_neighbours,
    bool only_orient) {
  #ifdef CGAL_LIB_EXISTS
    std::vector<Eigen::Vector3f> eigen_points(
      points->size(), Eigen::Vector3f(0, 0, 0));
    std::vector<Eigen::Vector3f> eigen_normals(
      points->size(), Eigen::Vector3f(0, 0, 0));
    for (size_t i = 0; i < points->size(); ++i) {
      eigen_points[i] = points->points[i].getArray3fMap();
      if (normals->size() == points->size()) 
        eigen_normals[i] = normals->points[i].getNormalVector3fMap();
    }
    computeAndOrientNormals(eigen_points, eigen_normals, n_neighbours, only_orient);
    normals->points.resize(points->size());
    for (size_t i = 0; i < normals->points.size(); ++i) {
      Eigen::Vector3f normal = eigen_normals[i];
      normals->points[i].normal[0] = normal[0];
      normals->points[i].normal[1] = normal[1];
      normals->points[i].normal[2] = normal[2];
    }
    return true;
  #else
    return false;
  #endif
}

void computeNormalsIfNecessary(pcl::PolygonMesh& mesh,
			 pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
			 pcl::PointCloud<pcl::Normal>::Ptr normals,
			 size_t n_neighbours,
       const pcl::search::Search<pcl::PointXYZ>::Ptr& tree,
       bool use_topology, bool persist_normals) {
  if (pcl::cloudHasNormals(mesh.cloud)) {
    pcl::fromPCLPointCloud2(mesh.cloud, *normals);
  } else {
    if (mesh.polygons.size() > 0 && use_topology) {
      computeNormals(mesh.polygons, points, normals);
    } else {
      computeNormals(points, normals, n_neighbours, tree);
      computeAndOrientNormalsWithCGAL(points, normals, n_neighbours, true);
    }
    if (persist_normals) {
      pcl::PCLPointCloud2 normals_cloud, current_cloud = mesh.cloud;
      pcl::toPCLPointCloud2(*normals, normals_cloud);
      pcl::concatenateFields(normals_cloud, current_cloud, mesh.cloud);
    }
  }
}

void computeFPFHs(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
	  pcl::PointCloud<pcl::Normal>::ConstPtr normals,
	  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs,
	  size_t n_neighbours, float neighbourhood_radius,
    const pcl::search::Search<pcl::PointXYZ>::Ptr& tree
) {
  pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
  fpfh.setInputCloud (points);
  fpfh.setInputNormals (normals);
  fpfh.setSearchMethod (tree);
  if (n_neighbours > 0) fpfh.setKSearch (n_neighbours);
  else fpfh.setRadiusSearch(neighbourhood_radius);

  fpfh.compute(*fpfhs);
}

void computeCustomFPFHs(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
    pcl::PointCloud<pcl::Normal>::ConstPtr normals,
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs,
    size_t n_neighbours, float neighbourhood_radius,
    const pcl::search::Search<pcl::PointXYZ>::Ptr& tree,
    pcl::IndicesPtr indices
) {
  /*pcl::CustomFPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
  fpfh.setInputCloud (points);
  fpfh.setInputNormals (normals);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  fpfh.setSearchMethod (tree);
  fpfh.setRadiusSearch (support_radius);
  fpfh.compute(*fpfhs);*/

  // std::vector<std::vector<size_t> > neighbours;
  // std::vector<std::vector<float> > neighbours_sqrdist;
  // computeNeighbours(points, neighbours, neighbours_sqrdist, (int)n_neighbours);
  // computeNeighbours(xyz_cloud, neighbours, neighbours_sqrdist, 30);

  if (indices.get() == 0) {
    indices = pcl::IndicesPtr(new std::vector<int>(points->size()));
    for (size_t i = 0; i < points->points.size(); ++i)
      (*indices)[i] = (i);
  } 

  std::vector<int> nn_indices (n_neighbours);
  std::vector<float> nn_dists (n_neighbours);

  int nr_bins_f1 = 11, nr_bins_f2 = 11, nr_bins_f3 = 11;
  int nr_bins = nr_bins_f1+nr_bins_f2+nr_bins_f3;

  Eigen::MatrixXf hist_f1; 
  Eigen::MatrixXf hist_f2; 
  Eigen::MatrixXf hist_f3;

  hist_f1.setZero (points->size(), nr_bins_f1);
  hist_f2.setZero (points->size(), nr_bins_f2);
  hist_f3.setZero (points->size(), nr_bins_f3);

  #pragma omp parallel for shared (hist_f1, hist_f2, hist_f3) private(nn_indices, nn_dists) num_threads(0)
  for (size_t s = 0;  s < indices->size(); ++s) {
    std::vector<int> tmp_nn_indices (n_neighbours);
    std::vector<float> tmp_nn_dists (n_neighbours);
    if (n_neighbours > 0) tree->nearestKSearch((*indices)[s], n_neighbours, tmp_nn_indices, nn_dists);
    else tree->radiusSearch((*indices)[s], neighbourhood_radius, tmp_nn_indices, nn_dists);
    for (size_t s2 = 0;  s2 < tmp_nn_indices.size(); ++s2) {
      size_t i = tmp_nn_indices[s2];
      if (hist_f1.row(i).sum() > 0 && hist_f2.row(i).sum() > 0 && hist_f3.row(i).sum() > 0)  continue;
      if (n_neighbours > 0) tree->nearestKSearch(i, n_neighbours, nn_indices, nn_dists);
      else tree->radiusSearch(i, neighbourhood_radius, nn_indices, nn_dists);

      float hist_incr = 1.0f / static_cast<float>(nn_indices.size() - 1);
      size_t idx1 = i;

      for (size_t k = 0; k < nn_indices.size(); ++k)
        if (nn_indices[k] != i  && nn_dists[k] > FLT_MIN) {
          size_t idx2 = nn_indices[k];
          Eigen::Vector4f f(0, 0, 0, 0); //alpha, psi, theta, d

          if(!pcl::computePairFeatures (
            (points->points[idx1]).getVector4fMap (), 
            (normals->points[idx1]).getNormalVector4fMap (),
            (points->points[idx2]).getVector4fMap (),
              (normals->points[idx2]).getNormalVector4fMap (),
              f[0], f[1], f[2], f[3]))  continue;
          f = f.cwiseAbs();

          // Normalize the f1, f2, f3 features and push them in the histogram
            int row = i;
            float d_pi_ = (1.0f / (2.0f * static_cast<float> (M_PI)));
            int h_index = static_cast<int> (floor (nr_bins_f1 * ((f[0] + M_PI) * d_pi_)));
            if (h_index < 0)           h_index = 0;
            if (h_index >= nr_bins_f1) h_index = nr_bins_f1 - 1;
            hist_f1 (row, h_index) += hist_incr;

            h_index = static_cast<int> (floor (nr_bins_f2 * ((f[1] + 1.0) * 0.5)));
            if (h_index < 0)           h_index = 0;
            if (h_index >= nr_bins_f2) h_index = nr_bins_f2 - 1;
            hist_f2 (row, h_index) += hist_incr;

            h_index = static_cast<int> (floor (nr_bins_f3 * ((f[2] + 1.0) * 0.5)));
            if (h_index < 0)           h_index = 0;
            if (h_index >= nr_bins_f3) h_index = nr_bins_f3 - 1;
            hist_f3 (row, h_index) += hist_incr;
        }
      }
  }

  std::cout << "Computed SPFH signatures.\n";
  fpfhs->points.resize(indices->size());

  std::cout << "Start computing FPFH signatures.\n";
  #pragma omp parallel for shared (hist_f1, hist_f2, hist_f3) private(nn_indices, nn_dists) num_threads(0)
  for (size_t s = 0;  s < indices->size(); ++s) {
    size_t i = (*indices)[s];
    if (n_neighbours > 0) tree->nearestKSearch(i, n_neighbours, nn_indices, nn_dists);
    else tree->radiusSearch(i, neighbourhood_radius, nn_indices, nn_dists);

    Eigen::RowVectorXf fpfh_histogram;
    fpfh_histogram.setZero(nr_bins);

    float sum_f1 = 0, sum_f2 = 0, sum_f3 = 0;

    for (size_t k = 0; k < nn_indices.size(); ++k)
      if (nn_indices[k] != i && nn_dists[k] > FLT_MIN) {
        size_t neigh_pt_id = nn_indices[k];
        float weight = 1.0f / (nn_dists[k]);

        for (int f1_i = 0; f1_i < nr_bins_f1; ++f1_i) {
          float val_f1 = hist_f1 (neigh_pt_id, f1_i) * weight;
          sum_f1 += val_f1;
          fpfh_histogram[f1_i] += val_f1;
        }

        for (int f2_i = 0; f2_i < nr_bins_f2; ++f2_i) {
          float val_f2 = hist_f2 (neigh_pt_id, f2_i) * weight;
          sum_f2 += val_f2;
          fpfh_histogram[f2_i + nr_bins_f1] += val_f2;
        }
        
        for (int f3_i = 0; f3_i < nr_bins_f3; ++f3_i) {
          float val_f3 = hist_f3 (neigh_pt_id, f3_i) * weight;
          sum_f3 += val_f3;
          fpfh_histogram[f3_i + nr_bins_f1 + nr_bins_f2] += val_f3;
        }
      }

    if (sum_f1 > FLT_MIN) sum_f1 = 100.0 / sum_f1;           // histogram values sum up to 100
    if (sum_f2 > FLT_MIN) sum_f2 = 100.0 / sum_f2;           // histogram values sum up to 100
    if (sum_f3 > FLT_MIN) sum_f3 = 100.0 / sum_f3;           // histogram values sum up to 100

     // Adjust final FPFH values
    for (int f1_i = 0; f1_i < nr_bins_f1; ++f1_i)
      fpfh_histogram[f1_i] *= static_cast<float> (sum_f1);
    for (int f2_i = 0; f2_i < nr_bins_f2; ++f2_i)
      fpfh_histogram[f2_i + nr_bins_f1] *= static_cast<float> (sum_f2);
    for (int f3_i = 0; f3_i < nr_bins_f3; ++f3_i)
      fpfh_histogram[f3_i + nr_bins_f1 + nr_bins_f2] *= static_cast<float> (sum_f3);

    for (size_t j = 0; j < fpfh_histogram.size(); ++j)
      fpfhs->points[s].histogram[j] = fpfh_histogram[j];
  }
}

void computeNormalsAndFPFHS(
    pcl::PolygonMesh& mesh,
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
    pcl::PointCloud<pcl::Normal>::Ptr normals,
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs,
    size_t n_neighbours, size_t n_descriptor_neighbours,
    const pcl::search::Search<pcl::PointXYZ>::Ptr& tree) {
  computeNormalsIfNecessary(mesh, points, normals, n_neighbours, tree);
  printf("computed normals.\n");
  computeCustomFPFHs(points, normals, fpfhs, n_descriptor_neighbours, 0.0f, tree);
  printf("computed descriptors.\n");
}

void computeOrientedNormalsAndFPFHS(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
    pcl::PointCloud<pcl::Normal>::Ptr normals,
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs,
    size_t n_neighbours, size_t n_descriptor_neighbours,
    const pcl::search::Search<pcl::PointXYZ>::Ptr& tree) {
  computeNormals(points, normals, n_neighbours, tree);
  printf("computed normals.\n");
  computeAndOrientNormalsWithCGAL(points, normals, n_neighbours, true);
  printf("oriented normals.\n");
  computeCustomFPFHs(points, normals, fpfhs, n_descriptor_neighbours, 0.0f, tree);
  printf("computed descriptors.\n");
}

void computeNormalsAndFPFHS(
    pcl::PolygonMesh& mesh,
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
    pcl::PointCloud<pcl::Normal>::Ptr normals,
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs,
    float neighbourhood_radius, float descriptor_radius,
    const pcl::search::Search<pcl::PointXYZ>::Ptr& tree) {
  int n_neighbours = computeAverageDensity(points, neighbourhood_radius, tree);
  computeNormalsIfNecessary(mesh, points, normals, n_neighbours, tree);
  printf("computed normals.\n");
  computeCustomFPFHs(points, normals, fpfhs, 0, descriptor_radius, tree);
  printf("computed descriptors.\n");
}

void computeOrientedNormalsAndFPFHS(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
    pcl::PointCloud<pcl::Normal>::Ptr normals,
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs,
    float neighbourhood_radius, float descriptor_radius,
    const pcl::search::Search<pcl::PointXYZ>::Ptr& tree) {
  int n_neighbours = computeAverageDensity(points, neighbourhood_radius, tree);
  computeNormals(points, normals, neighbourhood_radius, tree);
  printf("computed normals.\n");
  computeAndOrientNormalsWithCGAL(points, normals, n_neighbours, true);
  printf("oriented normals.\n");
  computeCustomFPFHs(points, normals, fpfhs, 0, descriptor_radius, tree);
  printf("computed descriptors.\n");
}

void computeSurfaceVariation(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
    pcl::PointCloud<pcl::Normal>::Ptr normals,
    std::vector<float>& surface_variation,
    int nb_neighbours,
    const pcl::search::Search<pcl::PointXYZ>::Ptr& tree
) {
  surface_variation.resize(points->size(), 0);
  std::vector<int> nn_indices(nb_neighbours);
  std::vector<float> nn_dists(nb_neighbours);

  #pragma omp parallel for shared (points, surface_variation) private(nn_indices, nn_dists)  num_threads(0)
  for (size_t i = 0; i < points->size(); ++i) {
    tree->nearestKSearch(i, nb_neighbours, nn_indices, nn_dists);

    Eigen::MatrixXf patch(nn_indices.size(), 3);
    for (size_t k = 0; k < nn_indices.size(); ++k)
      patch.row(k) = points->points[nn_indices[k]].getVector3fMap();
      // patch.row(k) = normals->points[neighbours[i][k]].getNormalVector3fMap();
    surface_variation[i] = compute_variation(patch, patch.colwise().mean());
  }
}

void computeSurfaceVariation(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
    pcl::PointCloud<pcl::Normal>::Ptr normals,
    std::vector<float>& surface_variation,
    float neighbourhood_radius,
    const pcl::search::Search<pcl::PointXYZ>::Ptr& tree) {

  surface_variation.resize(points->size(), 0);
  std::vector<int> nn_indices;
  std::vector<float> nn_dists;

  #pragma omp parallel for shared (points, surface_variation) private(nn_indices, nn_dists)  num_threads(0)
  for (size_t i = 0; i < points->size(); ++i) {
    tree->radiusSearch(i, neighbourhood_radius, nn_indices, nn_dists);

    Eigen::MatrixXf patch(nn_indices.size(), 3);
    for (size_t k = 0; k < nn_indices.size(); ++k)
      patch.row(k) = points->points[nn_indices[k]].getVector3fMap();
      // patch.row(k) = normals->points[neighbours[i][k]].getNormalVector3fMap();
    surface_variation[i] = compute_variation(patch, patch.colwise().mean());
  }
}

void createMatrixFromFPFHS(
    pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr fpfhs, 
    Eigen::MatrixXf& output) {
  output.setZero(fpfhs->size(), 33);

  #pragma omp parallel for shared (fpfhs, output)  num_threads(0)
  for (size_t i = 0; i < fpfhs->size(); ++i) {
    const Eigen::Map<const Eigen::RowVectorXf> descriptor(fpfhs->points[i].histogram, 33);
    output.row(i) = descriptor;
  }
}

} // pcl
