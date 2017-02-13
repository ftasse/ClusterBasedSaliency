#ifndef SALIENT_BASED_SIMPLIFICATION_H_
#define SALIENT_BASED_SIMPLIFICATION_H_

#include <vector>
#include <queue>
#include <set>
#include <float.h>

#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class SalientBasedSimplification
{
	struct PointPair
	{
		int id1, id2;
		int marker1, marker2;
		float cost;
		Eigen::Vector4f target;

		PointPair(int _id1 = -1, int _id2 = -1, float _cost = FLT_MAX, int _marker1 = 0, int _marker2 = 0):
			id1(_id1), id2(_id2), cost(_cost), marker1(_marker1), marker2(_marker2) 
		{}

		public:
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	struct point_pair_comparator{
	    bool operator() (const PointPair& p1, const PointPair& p2) const{
	        return p1.cost > p2.cost;
	    }
	};

public:
	SalientBasedSimplification();

	void collapse(
		int max_num_vertices = 1000
	);

	void compute(
		const pcl::PolygonMesh &mesh,
		const std::vector<float> &saliency,
		int max_num_vertices,
		pcl::PolygonMesh &simplified_mesh);

	void compute(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
		const pcl::PointCloud<pcl::Normal>::Ptr &normals,
		const std::vector<float> &saliency,
		int max_num_vertices,
		pcl::PointCloud<pcl::PointXYZ>::Ptr &simplified_cloud);

	void computeQuadrics(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
		const std::vector<pcl::Vertices> polygons,
		const std::vector<float> &saliency
	);

	void computeQuadrics(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
		const pcl::PointCloud<pcl::Normal>::Ptr &normals,
		const std::vector<float> &saliency
	);

	void construct_pairs(int id);

	void remove_point(int id);

	void prepare(int num_points);

	void getClusters(std::vector<std::vector<int> > &clusters, std::vector< std::vector<int> > &cluster_adjacency);

private:
	std::vector<PointPair, Eigen::aligned_allocator<PointPair> > collapsed_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr modified_cloud_;

	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > point_quadrics_;
	std::vector<int> point_markers_;
	std::vector<bool> point_status_;

	std::vector< std::set<int> > neighbours_;
	std::vector< std::set<int> > collapsed_indices_;

	std::priority_queue<PointPair, std::vector<PointPair, Eigen::aligned_allocator<PointPair> >, point_pair_comparator> point_pairs_;
};

#endif // SALIENT_BASED_SIMPLIFICATION_H_