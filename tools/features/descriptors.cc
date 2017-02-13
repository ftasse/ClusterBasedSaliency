#include "features/descriptors.h"

#include <pcl/common/time.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/pfh_tools.h>
#include <pcl/features/spin_image.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/point_traits.h>
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/PCLPointCloud2.h>

#include "features/normals.h"
#include "utils/pcl_utils.h"
#include "features/gabor.h"

// http://robotica.unileon.es/mediawiki/index.php/PCL/OpenNI_tutorial_4:_3D_object_recognition_%28descriptors%29

namespace pcl {

  bool computePolynomialCoeffs( 
      pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
      pcl::PointCloud<pcl::Normal>::ConstPtr normals,
      pcl::search::KdTree<pcl::PointXYZ>::ConstPtr tree,
      int index, float radius, int order,
      Eigen::VectorXd& coeffs) {
    int nr_coeff = (order + 1) * (order + 2) / 2;
    float sqr_gauss_param_ = radius*radius;
    coeffs  = Eigen::VectorXd::Zero(nr_coeff);

    std::vector<int> nn_indices;
    std::vector<float> nn_sqr_dists;
    tree->radiusSearch(index, radius, nn_indices, nn_sqr_dists);
    if (nn_indices.size() < nr_coeff) return false;
    // std::cout << "nn_indices: " << nn_indices.size() << "\n";
              
    // Compute the plane coefficients
    EIGEN_ALIGN16 Eigen::Matrix3d covariance_matrix;
    Eigen::Vector4d xyz_centroid;
    
    // Estimate the XYZ centroid
    pcl::compute3DCentroid (*points, nn_indices, xyz_centroid);
  
    // Compute the 3x3 covariance matrix
    pcl::computeCovarianceMatrix (*points, nn_indices, xyz_centroid, covariance_matrix);
    EIGEN_ALIGN16 Eigen::Vector3d::Scalar eigen_value;
    EIGEN_ALIGN16 Eigen::Vector3d eigen_vector;
    Eigen::Vector4d model_coefficients;
    pcl::eigen33 (covariance_matrix, eigen_value, eigen_vector);
    model_coefficients.head<3> ().matrix () = eigen_vector;
    // model_coefficients.head<3> ().matrix () = normals->points[index].getNormalVector3fMap();
    model_coefficients[3] = 0;
    model_coefficients[3] = -1 * model_coefficients.dot (xyz_centroid);

    // Projected query point
    Eigen::Vector3d point = points->points[index].getVector3fMap ().template cast<double> ();
    double distance = point.dot (model_coefficients.head<3> ()) + model_coefficients[3];
    point -= distance * model_coefficients.head<3> ();

    float curvature = static_cast<float> (covariance_matrix.trace ());
    // Compute the curvature surface change
    if (curvature != 0) 
      curvature = fabsf (float (eigen_value / double (curvature)));
  
    // Get a copy of the plane normal easy access
    Eigen::Vector3d plane_normal = model_coefficients.head<3> ();
    // Vector in which the polynomial coefficients will be put
    // Local coordinate system (Darboux frame)
    Eigen::Vector3d v_axis (0.0f, 0.0f, 0.0f), u_axis (0.0f, 0.0f, 0.0f);
  
    // Perform polynomial fit to update point and normal
    ////////////////////////////////////////////////////
    // Update neighborhood, since point was projected, and computing relative
    // positions. Note updating only distances for the weights for speed
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > de_meaned (nn_indices.size ());
    for (size_t ni = 0; ni < nn_indices.size (); ++ni) {
      de_meaned[ni][0] = points->points[nn_indices[ni]].x - point[0];
      de_meaned[ni][1] = points->points[nn_indices[ni]].y - point[1];
      de_meaned[ni][2] = points->points[nn_indices[ni]].z - point[2];
      nn_sqr_dists[ni] = static_cast<float> (de_meaned[ni].dot (de_meaned[ni]));
    }
    // Allocate matrices and vectors to hold the data used for the polynomial fit
    Eigen::VectorXd weight_vec (nn_indices.size ());
    Eigen::MatrixXd P (nr_coeff, nn_indices.size ());
    Eigen::VectorXd f_vec (nn_indices.size ());
    Eigen::MatrixXd P_weight; // size will be (nr_coeff_, nn_indices.size ());
    Eigen::MatrixXd P_weight_Pt (nr_coeff, nr_coeff);
  
    // Get local coordinate system (Darboux frame)
    v_axis = plane_normal.unitOrthogonal ();
    u_axis = plane_normal.cross (v_axis);
  
    // Go through neighbors, transform them in the local coordinate system,
    // save height and the evaluation of the polynome's terms
    double u_coord, v_coord, u_pow, v_pow;
    for (size_t ni = 0; ni < nn_indices.size (); ++ni) {
      // (Re-)compute weights
      weight_vec (ni) = exp (-nn_sqr_dists[ni] / sqr_gauss_param_);
      // Transforming coordinates
      u_coord = de_meaned[ni].dot (u_axis);
      v_coord = de_meaned[ni].dot (v_axis);
      f_vec (ni) = de_meaned[ni].dot (plane_normal);
      
      // Compute the polynomial's terms at the current point
      int j = 0;
      u_pow = 1;
      for (int ui = 0; ui <= order; ++ui) {
        v_pow = 1;
        for (int vi = 0; vi <= order - ui; ++vi) {
          P (j++, ni) = u_pow * v_pow;
          v_pow *= v_coord;
        }
        u_pow *= u_coord;
      }
    }
  
    // Computing coefficients
    P_weight = P * weight_vec.asDiagonal ();
    P_weight_Pt = P_weight * P.transpose ();
    coeffs = P_weight * f_vec;
    P_weight_Pt.llt ().solveInPlace (coeffs);
    return true;
  }

  void computeKeyPointDescriptors(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
    pcl::PointCloud<pcl::Normal>::ConstPtr normals,
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr keypoints_cloud,
    float radius, std::string descriptor_type,
    Eigen::MatrixXf& descriptors) {
      descriptors = Eigen::MatrixXf();
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud(points);

      if (descriptor_type == "fpfhs") {
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr signatures (new pcl::PointCloud<pcl::FPFHSignature33> ());
        pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> estimation;
        estimation.setInputCloud (keypoints_cloud);
        estimation.setSearchSurface(points);
        estimation.setInputNormals (normals);
        estimation.setSearchMethod (tree);
        estimation.setRadiusSearch (radius);
        estimation.compute (*signatures);

        descriptors.setZero(keypoints_cloud->points.size(), sizeof(signatures->points[0].histogram)/sizeof(float));
        for (unsigned int i = 0; i < signatures->size(); ++i)
          descriptors.row(i) = Eigen::Map<Eigen::RowVectorXf>(signatures->points[i].histogram, descriptors.cols())/100.0f;
      } else if (descriptor_type == "cfpfhs") {
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr signatures (new pcl::PointCloud<pcl::FPFHSignature33> ());
        computeCustomFPFHs(points, normals, signatures, 0, radius, tree);

        descriptors.setZero(keypoints_cloud->points.size(), sizeof(signatures->points[0].histogram)/sizeof(float));
        for (unsigned int i = 0; i < keypoints_cloud->size(); ++i) {
          std::vector<int> closest_point_id;
          std::vector<float> closest_point_dist;
          if (tree->nearestKSearch(keypoints_cloud->points[i], 1, closest_point_id, closest_point_dist)) {
            descriptors.row(i) = Eigen::Map<Eigen::RowVectorXf>(
              signatures->points[closest_point_id.front()].histogram, descriptors.cols())/100.0f;
          }
        }
      } else if (descriptor_type == "pfhs" || descriptor_type == "spatial_pfhs") {
        pcl::PointCloud<pcl::PFHSignature125>::Ptr signatures (new pcl::PointCloud<pcl::PFHSignature125> ());
        pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> estimation;
        estimation.setInputCloud (keypoints_cloud);
        estimation.setSearchSurface(points);
        estimation.setInputNormals (normals);
        estimation.setSearchMethod (tree);
        estimation.setRadiusSearch (radius);
        estimation.compute (*signatures);

        int pfh_dimension = sizeof(signatures->points[0].histogram)/sizeof(float);
        int dimension = (descriptor_type == "spatial_pfhs")?pfh_dimension+3:pfh_dimension;
        descriptors.setZero(keypoints_cloud->points.size(), dimension);

        Eigen::Vector4f centroid; 
        pcl::compute3DCentroid(*points, centroid);
        float length_x = 0, length_y = 0, length_z = 0;
        for (int i = 0; i < points->size(); ++i) {
          length_x = std::max(length_x, std::abs(points->points[i].x - centroid[0]));
          length_y = std::max(length_y, std::abs(points->points[i].y - centroid[1]));
          length_z = std::max(length_z, std::abs(points->points[i].z - centroid[2]));
        } 
        std::vector<std::pair<float,int> > coords(3);
        coords[0] = std::make_pair(length_x, 0);
        coords[1] = std::make_pair(length_y, 1);
        coords[2] = std::make_pair(length_z, 2);
        std::sort(coords.begin(), coords.end());

        for (unsigned int i = 0; i < signatures->size(); ++i) {
          Eigen::RowVectorXf descr (dimension); descr.setZero();
          descr.segment(0, pfh_dimension) = Eigen::Map<Eigen::RowVectorXf>(signatures->points[i].histogram, pfh_dimension)/100.0f;
          if (descriptor_type == "spatial_pfhs") {
            Eigen::Vector3f pt = keypoints_cloud->points[i].getVector3fMap();
            for (size_t k = 0; k < 3; ++k)
              descr(pfh_dimension+k) = std::abs((pt[coords[k].second] - centroid[coords[k].second])/coords[k].first);
          }
          descriptors.row(i) = descr;
        }
      } else if (descriptor_type == "cvfhs") {
        pcl::PointCloud<pcl::VFHSignature308>::Ptr signatures(new pcl::PointCloud<pcl::VFHSignature308>);
        pcl::CVFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> estimation;
        estimation.setInputCloud(keypoints_cloud);
        estimation.setSearchSurface(points);
        estimation.setInputNormals(normals);
        estimation.setSearchMethod(tree);
        estimation.setRadiusSearch (radius);
        estimation.setKSearch(0);
        // Set the maximum allowable deviation of the normals,
        // for the region segmentation step.
        estimation.setEPSAngleThreshold(5.0 / 180.0 * M_PI); // 5 degrees.
        // Set the curvature threshold (maximum disparity between curvatures),
        // for the region segmentation step.
        estimation.setCurvatureThreshold(1.0);
        // Set to true to normalize the bins of the resulting histogram,
        // using the total number of points. Note: enabling it will make CVFH
        // invariant to scale just like VFH, but the authors encourage the opposite.
        estimation.setNormalizeBins(false);
        estimation.compute (*signatures);

        descriptors.setZero(keypoints_cloud->points.size(), sizeof(signatures->points[0].histogram)/sizeof(float));
        for (unsigned int i = 0; i < signatures->size(); ++i)
          descriptors.row(i) = Eigen::Map<Eigen::RowVectorXf>(signatures->points[i].histogram, descriptors.cols());
      } else if (descriptor_type == "spinimages") {
        typedef pcl::Histogram<153> SpinImage;
        pcl::PointCloud<SpinImage>::Ptr signatures (new pcl::PointCloud<SpinImage> ());
        pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, SpinImage> estimation;
        estimation.setImageWidth(8);
        estimation.setInputCloud (points);
        estimation.setInputNormals (normals);
        estimation.setSearchMethod (tree);
        estimation.setRadiusSearch (radius);
        estimation.compute (*signatures);

        descriptors.setZero(keypoints_cloud->points.size(), sizeof(signatures->points[0].histogram)/sizeof(float));
        for (unsigned int i = 0; i < keypoints_cloud->size(); ++i) {
          std::vector<int> closest_point_id;
          std::vector<float> closest_point_dist;
          if (tree->nearestKSearch(keypoints_cloud->points[i], 1, closest_point_id, closest_point_dist)) {
            descriptors.row(i) = Eigen::Map<Eigen::RowVectorXf>(
              signatures->points[closest_point_id.front()].histogram, descriptors.cols());
          }
        }
      } else if (descriptor_type == "shot") {
        pcl::PointCloud<pcl::SHOT352>::Ptr signatures (new pcl::PointCloud<pcl::SHOT352> ());
        pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> estimation;
        estimation.setInputCloud (points);
        estimation.setInputNormals (normals);
        estimation.setSearchMethod (tree);
        estimation.setRadiusSearch (radius);
        estimation.compute (*signatures);

        descriptors.setZero(keypoints_cloud->points.size(), sizeof(signatures->points[0].descriptor)/sizeof(float));
        for (unsigned int i = 0; i < keypoints_cloud->size(); ++i) {
          std::vector<int> closest_point_id;
          std::vector<float> closest_point_dist;
          if (tree->nearestKSearch(keypoints_cloud->points[i], 1, closest_point_id, closest_point_dist)) {
            descriptors.row(i) = Eigen::Map<Eigen::RowVectorXf>(
              signatures->points[closest_point_id.front()].descriptor, descriptors.cols());
          }
        }
      } else if (descriptor_type == "gabor") {
        int num_scales = 3;
        int nr_split = 4;
        descriptors.setZero(keypoints_cloud->points.size(), num_scales*nr_split*nr_split*nr_split);

        pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> estimation;
        for (int c = 0; c < num_scales; ++c) {
          float sub_radius = ((c+1)*1.0/num_scales) * radius;  
          for (unsigned int i = 0; i < keypoints_cloud->size(); ++i) {
            std::vector<int> nn_indices;
            std::vector<float> nn_dists;
            if (tree->radiusSearch(keypoints_cloud->points[i], sub_radius, nn_indices, nn_dists)) {
              Eigen::VectorXf histogram(nr_split*nr_split*nr_split);
              estimation.computePointPFHSignature(*points, *normals, nn_indices, nr_split, histogram);
              descriptors.row(i).segment(c*histogram.size(), histogram.size()) = histogram;
            }
          }
        }
        // for (int c = 0; c < num_scales-1; ++c) {
        //   for (unsigned int i = 0; i < keypoints_cloud->size(); ++i) {
        //     descriptors.row(i).segment(c*nr_split*nr_split*nr_split, nr_split*nr_split*nr_split) = 
        //       descriptors.row(i).segment((c+1)*nr_split*nr_split*nr_split, nr_split*nr_split*nr_split) - 
        //       descriptors.row(i).segment(c*nr_split*nr_split*nr_split, nr_split*nr_split*nr_split);
        //   }
        // }
        // for (unsigned int i = 0; i < keypoints_cloud->size(); ++i)
        //   descriptors.row(i).segment((num_scales-1)*nr_split*nr_split*nr_split, nr_split*nr_split*nr_split).setZero();

      } else if (descriptor_type == "mls") {
        int order = 3;
        int num_scales = 1;
        int num_coeffs = (order + 1) * (order + 2) / 2;
        descriptors.setZero(keypoints_cloud->points.size(), num_scales*num_coeffs);

        for (int c = 0; c < num_scales; ++c) {
          float sub_radius = ((c+1)*1.0/num_scales) * radius;  

          for (unsigned int i = 0; i < keypoints_cloud->size(); ++i) {
            std::vector<int> closest_point_id;
            std::vector<float> closest_point_dist;
            if (tree->nearestKSearch(keypoints_cloud->points[i], 1, closest_point_id, closest_point_dist)) {
              Eigen::VectorXd histogram(num_coeffs);
              if (computePolynomialCoeffs(points, normals, tree, closest_point_id.front(), sub_radius, order, histogram)) {
                descriptors.row(i).segment(c*histogram.size(), histogram.size()) = histogram.template cast<float> ();
              }
            }
          }
        }
      } else if (descriptor_type == "covariance") {
        int num_scales = 3;
        int nr_features = 5; // nr_split*nr_split*nr_split;
        descriptors.setZero(keypoints_cloud->points.size(), num_scales*nr_features*nr_features);


        for (int c = 0; c < num_scales; ++c) {
          float sub_radius = ((c+1)*1.0/num_scales) * radius; // ((c+1)*1.0/num_scales)

          for (unsigned int i = 0; i < keypoints_cloud->size(); ++i) {
            std::vector<int> nn_indices;
            std::vector<float> nn_sqr_dists;
            if (tree->radiusSearch(keypoints_cloud->points[i], sub_radius, nn_indices, nn_sqr_dists)) {
              Eigen::MatrixXf local_descr = Eigen::MatrixXf::Zero (nn_indices.size(), nr_features);

              Eigen::Vector3f center(0, 0, 0);
              for (size_t k = 0; k < nn_indices.size(); ++k)
                center += points->points[nn_indices[k]].getVector3fMap();
              center /= nn_indices.size();

              for (size_t k = 0; k < nn_indices.size(); ++k) {
                Eigen::Vector3f point = points->points[nn_indices[k]].getVector3fMap();
                local_descr.row(k).segment(0, 3) = point - center;
                local_descr(k, 3) = sqrt(nn_sqr_dists[k]);
                local_descr(k, 4) = std::abs(Eigen::Vector3f(0,0,point[2]).dot(Eigen::Vector3f(0,point[1],0).cross(Eigen::Vector3f(point[0],0,0))));
              }

              Eigen::RowVectorXf mean = local_descr.colwise().mean();
              local_descr = local_descr.rowwise() - mean;
              Eigen::MatrixXf cov = (local_descr.transpose()*local_descr)/local_descr.rows();
              Eigen::Map<Eigen::RowVectorXf> cov_1D (cov.data(), cov.rows()*cov.cols());
              descriptors.row(i).segment(c*cov_1D.size(), cov_1D.size()) = cov_1D;
            }
          }
        }

        for (int c = 0; c < num_scales-1; ++c) {
          for (unsigned int i = 0; i < keypoints_cloud->size(); ++i) {
            descriptors.row(i).segment(c*nr_features*nr_features, nr_features*nr_features) = 
              descriptors.row(i).segment((c+1)*nr_features*nr_features, nr_features*nr_features) - 
              descriptors.row(i).segment(c*nr_features*nr_features, nr_features*nr_features);
          }
        }
      } else {
        printf("Error: invalid descriptor type: %s\n", descriptor_type.c_str());
      } 
  }

  void computeKeyPointDescriptors(
      pcl::PolygonMesh& mesh, pcl::PointCloud<pcl::PointXYZ>::Ptr keypoint_coords,
      float radius_percent, std::string descriptor_type, 
      Eigen::MatrixXf& descriptors, bool normals_from_topology, bool verbose) 
  {
    pcl::StopWatch watch;
    pcl::PointCloud<pcl::PointXYZ>::Ptr points (
      new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals (
      new pcl::PointCloud<pcl::Normal>);
    pcl::fromPCLPointCloud2(mesh.cloud, *points);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (points);
    if (!pcl::cloudHasNormals(mesh.cloud) && verbose) printf("Computing normals\n");
        computeNormalsIfNecessary(mesh, points, normals, 9, tree, normals_from_topology);

    float radius = radius_percent/getNormalizingScaleFactor(points, false);
    computeKeyPointDescriptors(points, normals, keypoint_coords,
      radius, descriptor_type, descriptors);
    
    if (verbose)
      pcl::console::print_highlight (
          "Computed %dx%d keypoint descriptors. Time: %f\n", 
          descriptors.rows(), descriptors.cols(), watch.getTimeSeconds());
  }
}