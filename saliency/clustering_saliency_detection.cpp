#include "saliency/clustering_saliency_detection.h"

#include <float.h>
#include <fstream>
#ifdef _OPENMP
  #include <omp.h>
#else
  #define omp_get_thread_num() 0
  #define omp_get_max_threads() 1
#endif
#ifdef USE_GNU_PARALLEL
#include <parallel/algorithm>
#endif

#include <Eigen/QR>

#include <flann/flann.hpp>
#include <flann/algorithms/dist.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>
#include <pcl/features/pfh.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/segmentation/extract_clusters.h>

// #include "tools/geometry/pcl_tools.h"
#include "tools/utils/eigen3_utils.h"
#include "tools/utils/pcl_utils.h"

// #include "tools/ml/gmm_fit.h"
// #include "tools/ml/gmm_fit_mlpack.h"
// #include "tools/ml/apcluster.h"

#include "segmentation/segmentation_utils.h"
// #include "segmentation/content_adaptive_clustering.h"


#define EIGEN_DONT_PARALLELIZE

typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> RowMajorMatrixXf;
typedef Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> RowMajorMatrixXi;

struct CustomTriplet {
	int i, j;
	double value;
	
	CustomTriplet(int _i = 0, int _j = 0, double _value = 0):
		i(_i), j(_j), value(_value)
	{}

	bool operator < (const CustomTriplet &other) const
	{
		if (j != other.j)
			return j < other.j;
		else
			return i < other.i;
	}
};

void computeDescriptorsKnn(
	const Eigen::MatrixXf &descriptors,
	RowMajorMatrixXi &neighbours,
	RowMajorMatrixXf &neighbours_dists,
	size_t max_n_neighbours);

ClusterBasedSaliencyDetection::ClusterBasedSaliencyDetection():
	normals_ (),
	histograms_(),
	searcher_ (),
	voxel_resolution_ (0.008f),
	use_adaptive_clustering_(true),
	clustering_fpfh_importance_(1.0),
	distribution_importance_(0.5),
	k_search_(0),
	radius_search_(0.0f),
	num_clusters_(0) {
	num_hist_bins_ = sizeof(HistogramT().histogram)/sizeof(float);
	// std::cout << "Num of bins per histogram: " << num_hist_bins_ << "\n\n";
}

ClusterBasedSaliencyDetection::~ClusterBasedSaliencyDetection() {
	if (searcher_ != 0)
		searcher_.reset ();
	if (normals_ != 0)
		normals_.reset ();
	if (histograms_ != 0)
		histograms_.reset ();
}

void ClusterBasedSaliencyDetection::setSearchMethod (const SearcherPtr& searcher) {
	if (searcher_ != 0)
		searcher_.reset ();
	searcher_ = searcher;
}

void ClusterBasedSaliencyDetection::setInputNormals (const NormalPtr& normals) {
	if (normals_ != 0)
		normals_.reset ();
	normals_ = normals;
}

Eigen::Map<Eigen::RowVectorXf> ClusterBasedSaliencyDetection::getPointDescriptor(int i) {
	return Eigen::Map<Eigen::RowVectorXf>(histograms_->points[i].histogram, num_hist_bins_);
}

void ClusterBasedSaliencyDetection::setInputHistograms (const HistogramPtr& histograms) {
	if (histograms_ != 0)
		histograms_.reset ();
	histograms_ = histograms;
}

ClusterBasedSaliencyDetection::NormalPtr
ClusterBasedSaliencyDetection::getInputNormals () const {
  return (normals_);
}

ClusterBasedSaliencyDetection::HistogramPtr
ClusterBasedSaliencyDetection::getInputHistograms () const {
  return (histograms_);
}

void ClusterBasedSaliencyDetection::computeSaliency(
		std::vector<float> &saliency,
		bool compute_per_point,
		IntermediateData *intermediate_data) {
	bool detection_is_possible = initCompute ();
	if ( !detection_is_possible || !prepare()) {
		deinitCompute ();
		return;
	}

	// Saliency detection
	pcl::StopWatch watch;
	float clustering_time = 0, probability_computation_time = 0;
	float clusterUD_computation_time = 0, cluster_saliency_time = 0;
	float saliency_assignment_time = 0;

	saliency.resize(input_->points.size(), 0.0);
	std::vector<int> labels(input_->points.size(), -1);
	std::vector<float> cluster_uniqueness, cluster_distribution;
	std::vector<float> cluster_saliency;

	oversegment();
	computeClusterInformation();
	for (size_t j = 0; j < segments_.size(); ++j)
		for (size_t k = 0; k < segments_[j].num_points(); ++k)
			labels[segments_[j].indexAt(k)] = j;

	clustering_time = watch.getTimeSeconds();
	pcl::console::print_highlight ("Computed supervoxel clustering. Time: %f\n", clustering_time);
	watch.reset();
	
	if (!compute_per_point) {
		probabilities_.resize(input_->points.size(), segments_.size());
		probabilities_.reserve(input_->points.size());
		for (size_t j = 0; j < segments_.size(); ++j) {
			for (size_t k =0; k < segments_[j].num_points(); ++k)
				probabilities_.insert(segments_[j].indexAt(k), j) = 1.0;
		}
		probabilities_.finalize();
		probabilities_.makeCompressed();
	}  else 
		computeClusterProbabilitiesFast(probabilities_);

	probability_computation_time = watch.getTimeSeconds();
	pcl::console::print_highlight (
		"Computed vertex-cluster probabilities (%d x %d). Time: %f\n", 
		histograms_->size(), segments_.size(),
		probability_computation_time);
	watch.reset();

	computeClusterUniquenessAndDistribution(cluster_uniqueness, cluster_distribution);

	clusterUD_computation_time = watch.getTimeSeconds();
	pcl::console::print_highlight ("Computed cluster uniqueness + distribution. Time: %f\n", clusterUD_computation_time);
	watch.reset();

	computeClusterSaliency(cluster_uniqueness, cluster_distribution, cluster_saliency);

	cluster_saliency_time = watch.getTimeSeconds();
	pcl::console::print_highlight ("Computed cluster saliency. Time: %f\n", cluster_saliency_time);
	watch.reset();

	normalizeFloatVector(cluster_saliency);
	for (int k=0; k<probabilities_.outerSize(); ++k)
		for (Eigen::SparseMatrix<double>::InnerIterator it(probabilities_,k); it; ++it) {
				float prob = it.value();
				int i = it.row();   // row index
				int j = it.col();   // col index (here it is equal to k)
				saliency[i] += prob * cluster_saliency[j];
			}

	float min_sal = *min_element(saliency.begin(), saliency.end());
    float max_sal = *max_element(saliency.begin(), saliency.end());
	printf("saliency min-max %f %f\n", min_sal, max_sal);

	normalizeFloatVector(saliency);

	saliency_assignment_time = watch.getTimeSeconds();
	pcl::console::print_highlight ("Computed per-vertex saliency. Time: %f\n", saliency_assignment_time);
	watch.reset();

	//Clean up
	cluster_centroid_.resize(0,3);
	cluster_mean_.resize(0,0);
	probabilities_.resize(0,0);

	if (intermediate_data) {
		intermediate_data->point_labels = labels;
		intermediate_data->cluster_uniqueness = cluster_uniqueness;
		intermediate_data->cluster_distribution = cluster_distribution;
		intermediate_data->cluster_saliency = cluster_saliency;
		intermediate_data->point_saliency = saliency;
		intermediate_data->cluster_weight = 
			std::vector<float>(cluster_weight_.data(), cluster_weight_.data()+segments_.size());

		intermediate_data->clustering_time = clustering_time;
		intermediate_data->probability_computation_time = probability_computation_time;
		intermediate_data->clusterUD_computation_time = clusterUD_computation_time;
		intermediate_data->cluster_saliency_time = cluster_saliency_time;
		intermediate_data->saliency_assignment_time = saliency_assignment_time;
		intermediate_data->setTotalTime();
	}
	cluster_weight_.resize(0);
	segments_.clear();
	segment_adjacency_.clear();
	deinitCompute ();
}

bool ClusterBasedSaliencyDetection::prepare () {
	if ( normals_ == 0 || input_->points.size () != normals_->points.size () )
    {
    	PCL_ERROR ("[ClusterBasedSaliencyDetection::prepare] Need to set input normals by using \"setInputNormals(const NormalPtr &normals)\".\n");
		return false;
    }

    if ( histograms_ == 0 || input_->points.size () != histograms_->points.size () )
    {
    	PCL_ERROR ("[ClusterBasedSaliencyDetection::prepare] Need to set input histograms by using \"setInputHistograms(const HistogramPtr &histograms)\".\n");
		return false;
    }

	if (!searcher_)
	{
		if (input_->isOrganized ())
			searcher_.reset (new pcl::search::OrganizedNeighbor<PointT> ());
		else
			searcher_.reset (new pcl::search::KdTree<PointT> ());
	}
	
	if (indices_)
	{
		if (indices_->empty ())
		  PCL_ERROR ("[ClusterBasedSaliencyDetection::prepare] Empty given indices!\n");
		if (!indices_->empty() || !searcher_->getInputCloud())
			searcher_->setInputCloud (input_, indices_);
	}
	else if (!searcher_->getInputCloud()) {
		searcher_->setInputCloud (input_);
	}

	return true;
}

void ClusterBasedSaliencyDetection::oversegment() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr original (new pcl::PointCloud<pcl::PointXYZ>());  
	/**original = *input_;
	std::vector<std::vector<size_t> > neighbours(input_->points.size());
	std::vector<std::vector<float> > neighbours_sqr_dists(input_->points.size());
	computeNeighbours(original, neighbours, neighbours_sqr_dists, k_search_);*/
	
	if (num_clusters_ >= input_->size()) {
		segments_.clear();
		for (size_t i = 0; i < input_->size(); ++i) {
			segments_.push_back(Segment());
			segments_.back().assignments.push_back(i);
		}
	} else { 
		if (!use_adaptive_clustering_) {
			classical_segment(input_, searcher_, segments_, input_->size()/num_clusters_, k_search_, radius_search_);
			// computeClustersRegionGrowing(original, normals_, segments_, num_clusters_, 9);
			// computeSuperClustersPapon(input_, normals_, histograms_,
		 //                            segments_, num_clusters_, 
		 //                            voxel_resolution_, k_search_, radius_search_, true, 0.0, 0.0, 0.0);
		}  else {
		  computeSuperClustersPapon(input_, normals_, histograms_,
		                            segments_, num_clusters_, 
		                            voxel_resolution_, k_search_, radius_search_, true, 1.0, 0.0, clustering_fpfh_importance_);
		}

		/*size_t min_density = (input_->size()/num_clusters_)*0.05;
		cleanup_segment(input_, neighbours, neighbours_sqr_dists, segments_,
	    							min_density*/
	}
	
	original.reset();
	//int size_thres = std::max(2, ((int)input_->points.size()) / 10000);
	//classical_segment(points, neighbours, neighbours_sqr_dists, segments_, size_thres); cleanup_segment(points, neighbours, neighbours_sqr_dists, segments_);
	pcl::computeSegmentAdjacencyWithNeighbourhood(segments_, searcher_, k_search_, radius_search_, segment_adjacency_);
	return;
}

void ClusterBasedSaliencyDetection::computeClusterInformation() {
	cluster_centroid_.setZero(segments_.size(), 3);
	cluster_mean_.setZero(segments_.size(), num_hist_bins_);
	cluster_weight_.setZero(segments_.size());
	is_cluster_outlier_ = std::vector<bool>(segments_.size(), false);

	pcl::PointCloud<pcl::PointXYZ>::Ptr centroids(
			new pcl::PointCloud<pcl::PointXYZ>);

	for (size_t j = 0; j < segments_.size(); ++j) {
		for (size_t k = 0; k < segments_[j].num_points(); ++k) {
			size_t pt_id = segments_[j].indexAt(k);
			cluster_centroid_.row(j) += Eigen::Vector3f(
				input_->points[pt_id].x,
				input_->points[pt_id].y,
				input_->points[pt_id].z);
			cluster_mean_.row(j) += getPointDescriptor(pt_id);
		}
		cluster_centroid_.row(j) /= segments_[j].num_points();
		cluster_mean_.row(j) /= segments_[j].num_points();
		cluster_weight_[j] = (segments_[j].num_points()*1.0) / input_->points.size();

		pcl::PointXYZ centroid;
		centroid.x = cluster_centroid_(j, 0);
		centroid.y = cluster_centroid_(j, 1);
		centroid.z = cluster_centroid_(j, 2);
		centroids->points.push_back(centroid);
	}

	// std::vector<int> outliers;
	// pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor(true);
 //  sor.setInputCloud (centroids);
 //  sor.setNegative(true);
 //  sor.setMeanK (4);
 //  sor.setStddevMulThresh (6);
 //  sor.filter (outliers);
 //  num_inlier_clusters_ = segments_.size() - outliers.size();
 //  std::cout << "Number of cluster outliers: " << outliers.size() << "\n";
 //  for (size_t i = 0; i < outliers.size(); ++i)	is_cluster_outlier_[outliers[i]] = true;

	
	cluster_total_mean_ = cluster_mean_.colwise().mean();
	cluster_total_centroid_ = cluster_centroid_.colwise().mean();

	cluster_total_spatial_variance_ = 0;
	cluster_total_geometric_variance_ = 0;

	// std::cout << cluster_total_mean_ << "\n" << cluster_total_centroid_ << "\n\n";
	for (size_t j = 0; j < segments_.size(); ++j) {
		Eigen::RowVectorXf centroid = cluster_centroid_.row(j);
		cluster_total_spatial_variance_ += flann::L2<float>()(cluster_total_centroid_.data(), centroid.data(), cluster_total_centroid_.size());

		Eigen::RowVectorXf mean = cluster_mean_.row(j);
		cluster_total_geometric_variance_ += flann::ChiSquareDistance<float>()(cluster_total_mean_.data(), mean.data(), cluster_total_mean_.size());
	}
	cluster_total_spatial_variance_ /= segments_.size();
	cluster_total_geometric_variance_ /= segments_.size();
}

float ClusterBasedSaliencyDetection::computeSpatialVariance(
	const std::vector<float>& scalar) {
	float var = 0;
	double mean, stddev;
	pcl::getMeanStdDev(scalar, mean, stddev);
	
	float z2 = 0.0;
	Eigen::RowVector3f mu;
	mu.setZero();	
	for (size_t j = 0; j < cluster_centroid_.rows(); ++j) if (!is_cluster_outlier_[j]) {
			float distance = (mean - scalar[j])*(mean - scalar[j]);
			float weight = 1/(1 + distance); 
			z2 += weight;
			mu += weight * cluster_centroid_.row(j); 
		}
	mu /= z2;
		
	z2 = 0.0;
	for (size_t j = 0; j < cluster_centroid_.rows(); ++j) if (!is_cluster_outlier_[j]) {
		float distance = (mean - scalar[j])*(mean - scalar[j]);
		float weight =  1/(1+distance); //exp(-descr_distance/descr_var);
		z2 += weight; 
		var += weight * (cluster_centroid_.row(j) - mu).squaredNorm();
	}
	
	return var/z2;
}

void ClusterBasedSaliencyDetection::computeClusterUniquenessAndDistribution(
	std::vector<float> &cluster_uniqueness,
	std::vector<float> &cluster_distribution) {
	cluster_uniqueness = std::vector<float>(cluster_centroid_.rows(), 0.0);
	cluster_distribution = std::vector<float>(cluster_centroid_.rows(), 0.0);

	int max_num_neighbours = segments_.size();

	std::cout << "Build FLANN index with FPFH\n";
	flann::Matrix<float> flann_fpfh_dataset(cluster_mean_.data(), cluster_mean_.rows(), cluster_mean_.cols());
	flann::IndexParams params = flann::LinearIndexParams();
	if (segments_.size() > 100) params = flann::KDTreeIndexParams(8);
	flann::Index<flann::ChiSquareDistance<float> > fpfh_index(flann_fpfh_dataset, params);
	fpfh_index.buildIndex();
	std::cout << "Built FLANN index with FPFH\n";
	/*flann::Matrix<float> flann_xyz_dataset(cluster_centroid_.data(), cluster_centroid_.rows(), cluster_centroid_.cols());
	flann::Index<flann::L2_Simple<float> > xyz_index(flann_xyz_dataset, flann::KDTreeIndexParams(5));
	xyz_index.buildIndex();
	std::cout << "Built FLANN index with XYZ\n";*/
	
	
	float spatial_var = cluster_total_spatial_variance_;
	float descr_var = cluster_total_geometric_variance_;
	std::cout << "Variances: " << spatial_var << " " << descr_var << "\n";

	float min_uniqueness = FLT_MAX;
	float max_distribution = FLT_MIN;
	float max_uniqueness = FLT_MIN;
	float min_distribution = FLT_MAX;

	// std::vector<std::map<int, float> > neighbours_dists_vec(segments_.size());
	// int nlevels = pow(segments_.size(), 1.0/3.0);
	// #pragma omp parallel for default(shared)
	// for (int j = 0; j < cluster_centroid_.rows(); ++j)
	// 	getSegmentNeighbourhood(j, nlevels, 0, neighbours_dists_vec[j]);

	flann::Matrix<float> fpfh_query;
	flann::Matrix<int> nn_indices_fpfh;
	flann::Matrix<float> nn_dists_fpfh;
	flann::Matrix<float> nn_dists_xyz;
	/*flann::Matrix<float> xyz_query(new float[3], 1, 3);
	flann::Matrix<int> nn_indices_xyz(new int[xyz_query.rows*max_num_neighbours], xyz_query.rows, max_num_neighbours);
	flann::Matrix<float> nn_dists_xyz(new float[xyz_query.rows*max_num_neighbours], xyz_query.rows, max_num_neighbours);*/

	//#ifdef EIGEN_DONT_PARALLELIZE
	#pragma omp parallel for default(shared) private(fpfh_query, nn_indices_fpfh, nn_dists_fpfh, nn_dists_xyz)
	//#endif
	for (int j = 0; j < cluster_centroid_.rows(); ++j) {
		if (is_cluster_outlier_[j]) continue;

		fpfh_query = flann::Matrix<float>(new float[cluster_mean_.cols()], 1, cluster_mean_.cols());
		nn_indices_fpfh = flann::Matrix<int>(new int[fpfh_query.rows*max_num_neighbours], fpfh_query.rows, max_num_neighbours);
		nn_dists_fpfh = flann::Matrix<float>(new float[fpfh_query.rows*max_num_neighbours], fpfh_query.rows, max_num_neighbours);
		nn_dists_xyz = flann::Matrix<float>(new float[fpfh_query.rows*max_num_neighbours], fpfh_query.rows, max_num_neighbours);

		std::map<int, int> cluster_id_to_rank;

		for (int l = 0; l < cluster_mean_.cols(); ++l) 
			fpfh_query[0][l] = cluster_mean_(j, l);
		for (int k = 0; k < max_num_neighbours; ++k)
			nn_indices_fpfh[0][k] = -1;

		float max_fpfh_dist = 0, max_xyz_dist = 0;

		fpfh_index.knnSearch(fpfh_query, nn_indices_fpfh, nn_dists_fpfh,
							 max_num_neighbours, flann::SearchParams());
		for (int k = 0; k < nn_indices_fpfh.cols; ++k) 
			if (nn_indices_fpfh[0][k] >= 0 && nn_indices_fpfh[0][k] < cluster_centroid_.rows()) {
				nn_dists_xyz[0][k] = 
					(cluster_centroid_.row(j) - cluster_centroid_.row(nn_indices_fpfh[0][k]))
					.squaredNorm();
				cluster_id_to_rank[nn_indices_fpfh[0][k]] = k;
				max_xyz_dist = std::max(nn_dists_xyz[0][k], max_xyz_dist);
				max_fpfh_dist = std::max(nn_dists_fpfh[0][k], max_fpfh_dist);
			}
			else	break;

		// std::map<int, float>& neighbours_dists = neighbours_dists_vec[j];

		cluster_uniqueness[j] = 0.0;
		cluster_distribution[j] = 0.0;
		
		float z = 0.0;
		for (size_t jj = 0; jj < nn_indices_fpfh.cols; ++jj)
			if (nn_indices_fpfh[0][jj] >= 0 && nn_indices_fpfh[0][jj] < cluster_centroid_.rows()) {
				// if (neighbours_dists.find(nn_indices_fpfh[0][jj])==neighbours_dists.end()) continue;
				size_t neigh_pt_id = nn_indices_fpfh[0][jj];
				if (is_cluster_outlier_[neigh_pt_id]) continue;
				float descr_distance = nn_dists_fpfh[0][jj];
				float spatial_distance = nn_dists_xyz[0][jj]; // neighbours_dists[neigh_pt_id]; //nn_dists_xyz[0][jj]; // 
				float weight = 1/(1 + spatial_distance); // exp(-spatial_distance/spatial_var); // 1/(1 + spatial_distance); 
				z += weight; 
				cluster_uniqueness[j] += cluster_weight_[neigh_pt_id] * weight * descr_distance; // cluster_weight_[neigh_pt_id] * 
				// cluster_uniqueness[neigh_pt_id] = 1 + descr_distance;
			} else break;
		cluster_uniqueness[j] /= z; 

		float z2 = 0.0;
		float dist_thres = max_xyz_dist*0.5;
		float fpfh_thres = max_fpfh_dist*0.5;
		int jj_thres = cluster_distribution.size()/10;
		Eigen::RowVector3f mu(0, 0, 0);
		for (size_t jj = 0; jj < nn_indices_fpfh.cols; ++jj)
			if (nn_indices_fpfh[0][jj] >= 0 && nn_indices_fpfh[0][jj] < cluster_centroid_.rows())  {
				// if (nn_dists_xyz[0][jj] > dist_thres) continue;
				// if (nn_dists_fpfh[0][jj] > fpfh_thres) continue;
				// if (jj > jj_thres) continue;
				size_t neigh_pt_id = nn_indices_fpfh[0][jj];
				if (is_cluster_outlier_[neigh_pt_id]) continue;
				float descr_distance = nn_dists_fpfh[0][jj];
				float spatial_distance = nn_dists_xyz[0][jj];
				float weight =  exp(-descr_distance/descr_var);
				z2 += weight; 
				mu += weight * cluster_centroid_.row(neigh_pt_id);
			} else break;
		mu /= z2;

		z2 = 0.0;
		for (size_t jj = 0; jj < nn_indices_fpfh.cols; ++jj)
			if (nn_indices_fpfh[0][jj] >= 0 && nn_indices_fpfh[0][jj] < cluster_centroid_.rows())  {
				// if (nn_dists_xyz[0][jj] > dist_thres) continue;
				// if (nn_dists_fpfh[0][jj] > fpfh_thres) continue;
				// if (jj > jj_thres) continue;
				size_t neigh_pt_id = nn_indices_fpfh[0][jj];
				if (is_cluster_outlier_[neigh_pt_id]) continue;
				float descr_distance = nn_dists_fpfh[0][jj];
				float spatial_distance = nn_dists_xyz[0][jj];
				float weight =  exp(-descr_distance/descr_var);
				z2 += weight; 
				cluster_distribution[j] += cluster_weight_[neigh_pt_id] * weight * (cluster_centroid_.row(neigh_pt_id) - mu).squaredNorm();
			} else break;
		cluster_distribution[j] /= z2;

		min_uniqueness = std::min(min_uniqueness, cluster_uniqueness[j]);
		max_uniqueness = std::max(max_uniqueness, cluster_uniqueness[j]);
		min_distribution = std::min(min_distribution, cluster_distribution[j]);
		max_distribution = std::max(max_distribution, cluster_distribution[j]);

		delete [] fpfh_query.ptr();
		delete [] nn_indices_fpfh.ptr();
		delete [] nn_dists_fpfh.ptr();
		delete [] nn_dists_xyz.ptr();
	}

	std::cout << "Min uniqueness and distribution: " << min_uniqueness << " " << min_distribution << "\n";
	for (int j = 0; j < segments_.size(); ++j) if (is_cluster_outlier_[j]) {
		cluster_uniqueness[j] = min_uniqueness;
		cluster_distribution[j] = max_distribution;
	}

	// removeOutliers(cluster_distribution, -2, 2);
	// removeOutliers(cluster_uniqueness, -2, 2);

	normalizeFloatVector(cluster_distribution);
	normalizeFloatVector(cluster_uniqueness);

	for (size_t j = 0; j < cluster_centroid_.rows(); ++j) 
		cluster_distribution[j] = exp(-cluster_distribution[j]);
	for (size_t j = 0; j < cluster_centroid_.rows(); ++j) 
		cluster_uniqueness[j] = 1-exp(-cluster_uniqueness[j]);

	return;
}

struct FrontierPoint {
	int cluster_id;
	int level;
	float dist;
	FrontierPoint(int cluster_id_, int level_, int dist_):cluster_id(cluster_id_), level(level_), dist(dist_) {}
};

void ClusterBasedSaliencyDetection::getSegmentNeighbourhood(int j, int nlevels, int max_num, std::map<int, float>& neighbours) {
	neighbours.clear();
	int stop_level = segments_.size();
			
	if (!is_cluster_outlier_[j]) {
		std::queue< FrontierPoint > frontier; //has cluster id + neighbourhood level
		frontier.push(FrontierPoint(j, 0, 0));
		while(!frontier.empty()) {
			FrontierPoint current = frontier.front();
			frontier.pop();
			int current_id = current.cluster_id;
			int current_level = current.level;
			float dist = current.dist;

			if (is_cluster_outlier_[current_id]) continue;
			else {
				std::map<int, float>::iterator it  = neighbours.find(current_id);
				if (it == neighbours.end()) {
					if (current_level <= stop_level)
					neighbours[current_id] = dist;
				}
				else if (it->second > dist) neighbours[current_id] = dist; 
			}

			if (current_level > 0 && (max_num > 0 && neighbours.size() >= max_num)) {
				stop_level = current_level;
				continue;
			}

			if (nlevels == 0 || current_level < nlevels)
				for (size_t k =0; k < segment_adjacency_[current_id].size(); ++k) {
					int jj = segment_adjacency_[current_id][k];
					if (neighbours.find(jj) == neighbours.end())
						frontier.push(FrontierPoint(jj, current_level+1, dist + (cluster_centroid_.row(current_id) - cluster_centroid_.row(jj)).squaredNorm()));
				}
		}
	}
}

void ClusterBasedSaliencyDetection::computeClusterProbabilitiesFast(
	Eigen::SparseMatrix<double> &probabilities ) {
	pcl::StopWatch watch;
	int nlevels = 3;

	size_t avg_seg_neighbourhood_size = 9 * nlevels;
	
	std::vector<CustomTriplet> coefficients;
	coefficients.reserve(input_->points.size()*avg_seg_neighbourhood_size);

	// std::cout <<  << " " << "compute weights\n";
	Eigen::SparseMatrix<double> &weights = probabilities;
	weights.resize(histograms_->size(), segments_.size());
	// weights.reserve(input_->size()*avg_seg_neighbourhood_size);

	//#pragma omp parallel for default(shared)
	#pragma omp parallel default(shared)
    {
	  std::vector<CustomTriplet> coefficients_private; 
	  coefficients_private.reserve((input_->size()/segments_.size())*avg_seg_neighbourhood_size*(segments_.size()/omp_get_max_threads()));
	  
	  #pragma omp for nowait //fill coefficients_private in parallel
		for (size_t j = 0; j < segments_.size(); ++j) {
			// std::sort(segments_[j].assignments.begin(), segments_[j].assignments.end());
			std::vector<int> neighbours;
			std::map<int, float> neighbours_dists;
			getSegmentNeighbourhood(j, nlevels, 0, neighbours_dists);
			for (std::map<int, float>::iterator it = neighbours_dists.begin(); it != neighbours_dists.end(); ++it)
				neighbours.push_back(it->first);
			
			if (neighbours.size() <= 1) {
				for (size_t k = 0; k < segments_[j].assignments.size(); ++k)
					coefficients_private.push_back(
						CustomTriplet(segments_[j].assignments[k], j, 1.0));
				continue;
			}
			
			///////////////////////////////////////////////////////
			// std::cout << j << " " << "compute weights\n";
		
			const std::vector<int> &seg_points = segments_[j].assignments;

			Eigen::MatrixXf neighbours_mean(neighbours.size(), num_hist_bins_);
			Eigen::MatrixX3f neighbours_centroid(neighbours.size(), 3);

			for (int k = 0; k < neighbours.size(); ++k) {
				int jj = neighbours[k];
				neighbours_mean.row(k) = cluster_mean_.row(jj);
				neighbours_centroid.row(k) = cluster_centroid_.row(jj);
			}

			Eigen::MatrixX3f seg_points_mat(seg_points.size(), 3);
			Eigen::MatrixXf seg_points_descr(seg_points.size(), num_hist_bins_);
			for (size_t k = 0; k < seg_points.size(); ++k) {
				seg_points_mat.row(k) = input_->points[seg_points[k]].getVector3fMap();
				seg_points_descr.row(k) = getPointDescriptor(seg_points[k]);
			}

			Eigen::MatrixXd descr_distances(seg_points.size(), neighbours_mean.rows());	
			Eigen::MatrixXd spatial_distances(seg_points.size(), neighbours_mean.rows());	

			// #ifdef EIGEN_DONT_PARALLELIZE
			// #pragma omp parallel for default(shared)
			// #endif
			for (int k = 0; k < neighbours_mean.rows(); ++k) {
				const Eigen::RowVector3f &row = neighbours_centroid.row(k);
				spatial_distances.col(k) = (seg_points_mat.rowwise() - row).rowwise().squaredNorm().cast<double>();
			}

			// #ifdef EIGEN_DONT_PARALLELIZE
			// #pragma omp parallel for default(shared)
			// #endif
			for (int k = 0; k < neighbours_mean.rows(); ++k) {
				// Compute chi-square distance
				const Eigen::RowVectorXf &row = neighbours_mean.row(k);
				/*for (size_t c = 0; c < seg_points_descr.rows(); ++c) {
					Eigen::RowVectorXf vec1 = neighbours_mean.row(k);
					Eigen::RowVectorXf vec2 = seg_points_descr.row(c);
					float dist = flann::ChiSquareDistance<float>()(vec1.data(), vec2.data(), vec1.size());
					descr_distances(c,k) = dist;
				}*/
				Eigen::ArrayXXf A = (seg_points_descr.rowwise() - row).array().square();
				Eigen::ArrayXXf B = (seg_points_descr.rowwise() + row).array() + FLT_MIN;
				descr_distances.col(k) = (A / B ).rowwise().sum().cast<double>();
			}

			float overall_spatial_var = cluster_total_spatial_variance_;
			float overall_descr_var = cluster_total_geometric_variance_;
			float a = (1.0/overall_spatial_var);
			float b = (5.0/overall_descr_var);
			Eigen::MatrixXd intermediate_weights = (-0.5*a*spatial_distances - 0.5*b*descr_distances).array().exp();

			for (size_t k = 0; k < seg_points.size(); ++k) {
				int i = seg_points[k];
				for (size_t kk = 0; kk < neighbours_mean.rows(); ++kk) {
					int neigh_j = neighbours[kk];
					coefficients_private.push_back(CustomTriplet(i, neigh_j, intermediate_weights(k, kk)));
				}
			}
		}

	  #pragma omp critical
	  	coefficients.insert(coefficients.end(), coefficients_private.begin(), coefficients_private.end());
	}

	std::cout << "Computed weights " << weights.rows() << " " << weights.cols() << "\n";

	// For older eigen versions, make sure j is increasing, followed by i
	#ifdef USE_GNU_PARALLEL
	__gnu_parallel::sort(coefficients.begin(), coefficients.end());
	#else
	std::sort(coefficients.begin(), coefficients.end());
	#endif
	weights.reserve(coefficients.size());
	int cur_j = -1;
	for (int k = 0; k < coefficients.size(); ++k) {
		if (coefficients[k].j != cur_j) {
			cur_j = coefficients[k].j;
			weights.startVec(cur_j);
		}
		weights.insertBack(coefficients[k].i,coefficients[k].j) = coefficients[k].value;
		// if (coefficients[k].i == 1596) std::cout << "*******" << " " << coefficients[k].j << " " << coefficients[k].value << " " << weights.coeffRef(coefficients[k].i, coefficients[k].j) << "\n";
	}
	coefficients.clear();
	weights.finalize();
	weights.makeCompressed();

	std::cout << avg_seg_neighbourhood_size*input_->size() << " " << weights.nonZeros() << " filled in matrix\n";

	Eigen::RowVectorXd tmp_weights (segments_.size());
	// std::cout << weights.nonZeros() << " filled in matrix\n";
	#pragma omp parallel for default(shared)
	for (size_t j = 0; j < segments_.size(); ++j)
		tmp_weights[j] = weights.col(j).sum();
	cluster_weight_ = (tmp_weights / tmp_weights.sum()).cast<float>();
	
	#pragma omp parallel for default(shared)
	for (int k=0; k<weights.outerSize(); ++k)
		for (Eigen::SparseMatrix<double>::InnerIterator it(weights,k); it; ++it)
			if (tmp_weights[it.col()] > FLT_MIN)
				it.valueRef() /= tmp_weights[it.col()];

	Eigen::VectorXd tmp_weights2 = Eigen::VectorXd::Zero(probabilities.rows());
	#pragma omp parallel default(shared) 
	{
	    Eigen::VectorXd tmp_weights2_private = Eigen::VectorXd::Zero(probabilities.rows());
	    #pragma omp for
	    for (int k=0; k<weights.outerSize(); ++k)
			for (Eigen::SparseMatrix<double>::InnerIterator it(weights,k); it; ++it)
				tmp_weights2_private[it.row()] += it.value();
	    #pragma omp critical
	    {
	        for(size_t i=0; i < tmp_weights2.size(); ++i) {
	            tmp_weights2[i] += tmp_weights2_private[i];
	        }
	    }
	}
	weights = tmp_weights2.asDiagonal().inverse() * weights;

	// std::cout << "Highest probability for point 0: " << probabilities.row(1).maxCoeff() << "\n";
	pcl::console::print_info ("vertex-cluster probabilities - computed probabilities. Time: %f\n",  watch.getTimeSeconds());
}

void ClusterBasedSaliencyDetection::computeClusterSaliency(
	const std::vector<float> &cluster_uniqueness,
	const std::vector<float> &cluster_distribution,
	std::vector<float> &cluster_saliency) {
	cluster_saliency.resize(cluster_uniqueness.size());

	double uniqueness_mean, uniqueness_stddev;
	double distribution_mean, distribution_stddev;
	// pcl::getMeanStdDev(cluster_distribution, distribution_mean, distribution_stddev);
	// pcl::getMeanStdDev(cluster_uniqueness, uniqueness_mean, uniqueness_stddev);
	uniqueness_stddev = computeSpatialVariance(cluster_uniqueness);
	distribution_stddev = computeSpatialVariance(cluster_distribution);
	std::cout << "Saliency variances: " << uniqueness_stddev << " " << distribution_stddev << "\n";

	for (size_t j = 0; j < cluster_uniqueness.size(); ++j) {
		// if (uniqueness_stddev < distribution_stddev) cluster_saliency[j] = cluster_uniqueness[j];
		// else cluster_saliency[j] = cluster_distribution[j];
		cluster_saliency[j] = distribution_importance_*(cluster_distribution[j]) + 
												 (1-distribution_importance_)*(cluster_uniqueness[j]); // bad for unclean data like T8.off
	}
}

