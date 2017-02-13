#ifndef CLUSTERING_SALIENCY_DETECTION_H
#define CLUSTERING_SALIENCY_DETECTION_H

#include <Eigen/Sparse>
#include <pcl/pcl_base.h>
#include <pcl/search/pcl_search.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>

#include "segmentation/segment.h"

typedef pcl::PointXYZ PointT;
typedef pcl::Normal NormalT;
typedef pcl::FPFHSignature33 HistogramT;

class ClusterBasedSaliencyDetection : public pcl::PCLBase<PointT>
{
public:
	struct IntermediateData {
		std::vector<int> point_labels;
		std::vector<float> cluster_uniqueness;
		std::vector<float> cluster_distribution;
		std::vector<float> cluster_saliency;
		std::vector<float> cluster_weight;
		std::vector<float> point_saliency;

		float clustering_time, probability_computation_time;
		float clusterUD_computation_time, cluster_saliency_time;
		float saliency_assignment_time;
		float descriptor_computation_time;

		float total_time;

		IntermediateData(){}

		float getTotalTimeInSecs() const
		{
			float secs = clustering_time + probability_computation_time;
			secs += clusterUD_computation_time + cluster_saliency_time;
			secs += saliency_assignment_time + descriptor_computation_time;
			return secs;
		}

		void setTotalTime()
		{
			total_time = getTotalTimeInSecs();
		}
	};

protected:
	typedef pcl::PointCloud<PointT> PointCloudT;
	typedef pcl::PointCloud<NormalT> NormalCloudT;
	typedef pcl::PointCloud<HistogramT> HistogramCloudT;
	typedef NormalCloudT::Ptr NormalPtr;
	typedef HistogramCloudT::Ptr HistogramPtr;
	typedef pcl::search::Search<PointT>::Ptr SearcherPtr;
	
	using PCLBase <PointT>::initCompute;
	using PCLBase <PointT>::deinitCompute;
	using PCLBase <PointT>::input_;
	using PCLBase <PointT>::indices_;

public:
	ClusterBasedSaliencyDetection();
	virtual ~ClusterBasedSaliencyDetection();

	void setSearchMethod (const SearcherPtr &searcher);
	void setInputNormals (const NormalPtr& norm);
	void setInputHistograms (const HistogramPtr& hists);
	NormalPtr getInputNormals () const;
	HistogramPtr getInputHistograms () const;

	void setClusteringResolution(float voxel_resolution) {
		voxel_resolution_ = voxel_resolution;
	}

	void setClusterSize(int num_clusters)
	{
		num_clusters_ = num_clusters;
	}

	void useAdaptiveClustering(bool use_adaptive_clustering) {
		use_adaptive_clustering_ = use_adaptive_clustering;
	}

	void setDistributionImportance(float distribution_importance) {
		distribution_importance_ = distribution_importance;
	}

	void setClusteringFPFHImportance(float clustering_fpfh_importance) {
		clustering_fpfh_importance_ = clustering_fpfh_importance;
	}

	void setKSearch(int k_search) {
		k_search_ = k_search;
	}

	void setRadiusSearch(float radius_search) {
		radius_search_ = radius_search;
	}

	virtual void computeSaliency(
		std::vector<float> &saliency,
		bool compute_per_point = true,
		IntermediateData *intermediate_data = NULL);

protected:
	virtual bool prepare();
	virtual void oversegment();

	virtual void computeClusterInformation();

	virtual void computeClusterUniquenessAndDistribution(
		std::vector<float> &cluster_uniqueness,
		std::vector<float> &cluster_distribution
	);

	virtual float computeSpatialVariance(const std::vector<float>& scalar);

	virtual void computeClusterSaliency(
		const std::vector<float> &cluster_uniqueness,
		const std::vector<float> &cluster_distribution,
		std::vector<float> &cluster_saliency
	);

	Eigen::Map<Eigen::RowVectorXf> getPointDescriptor(int i);

	virtual void getSegmentNeighbourhood(int j, int nlevels, int max_num, std::map<int, float>& neighbours);

	virtual void computeClusterProbabilitiesFast(Eigen::SparseMatrix<double> &probabilities);

	std::vector< Segment > segments_;
	std::vector< std::vector<int> > segment_adjacency_;

	Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> cluster_centroid_;
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> cluster_mean_;
	Eigen::VectorXf cluster_weight_;

	Eigen::RowVectorXf cluster_total_mean_;
	Eigen::RowVectorXf cluster_total_centroid_;
	float cluster_total_spatial_variance_;
	float cluster_total_geometric_variance_;

	std::vector<bool> is_cluster_outlier_;
	int num_inlier_clusters_;

	Eigen::SparseMatrix<double> probabilities_;

private:
	NormalPtr normals_;
	HistogramPtr histograms_;
	SearcherPtr searcher_;

	int num_hist_bins_;

	float voxel_resolution_;
	int num_clusters_;
	bool use_adaptive_clustering_;
	float clustering_fpfh_importance_;
	float distribution_importance_;
	int k_search_;
	float radius_search_;
};

#endif // CLUSTERING_SALIENCY_DETECTION_H