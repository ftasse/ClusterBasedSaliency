#include "simplification/salient_based_simplification.h"

#include <pcl/conversions.h>
#include <pcl/search/pcl_search.h>


#include "utils/he_mesh.h"

SalientBasedSimplification::SalientBasedSimplification()
{

}

void SalientBasedSimplification::compute(
	const pcl::PolygonMesh &mesh,
	const std::vector<float> &saliency,
	int max_num_vertices,
	pcl::PolygonMesh &simplified_mesh)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr simplified_cloud(new pcl::PointCloud<pcl::PointXYZ>());

	pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromPCLPointCloud2(mesh.cloud, *xyz_cloud);

	if (mesh.polygons.size() > 0)
	{

		neighbours_.clear(); neighbours_.resize(xyz_cloud->points.size());

		for (unsigned int i = 0; i < mesh.polygons.size(); ++i)
		{
			for (unsigned int j = 0; j < mesh.polygons[i].vertices.size(); ++j)
			{
				int id1 = mesh.polygons[i].vertices[j];
				int id2 = mesh.polygons[i].vertices[(j+1)% mesh.polygons[i].vertices.size()];
				neighbours_[id2].insert(id1);
				neighbours_[id1].insert(id2);
			}
		}

		computeQuadrics(xyz_cloud, mesh.polygons, saliency);
		modified_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
		*modified_cloud_ = *xyz_cloud;
		collapse(max_num_vertices);

		HEMesh he_mesh(*modified_cloud_, mesh.polygons);
		for (unsigned int i = 0; i < collapsed_.size(); ++i)
			he_mesh.collapse_edge(collapsed_[i].id1, collapsed_[i].id2);
		he_mesh.convertTo(*simplified_cloud, simplified_mesh.polygons);
	} else
	{
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
		pcl::fromPCLPointCloud2(mesh.cloud, *normals);
		compute(xyz_cloud, normals, saliency, max_num_vertices, simplified_cloud);
	}

	pcl::toPCLPointCloud2(*simplified_cloud, simplified_mesh.cloud);

	/*int test;
	std::cin >> test;*/
}

void SalientBasedSimplification::compute(
	const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
	const pcl::PointCloud<pcl::Normal>::Ptr &normals,
	const std::vector<float> &saliency,
	int max_num_vertices,
	pcl::PointCloud<pcl::PointXYZ>::Ptr &simplified_cloud)
{
	neighbours_.clear(); neighbours_.resize(cloud->points.size());

	int nnk = 6;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);

	std::vector<int> pointIdxRadiusSearch;
  	std::vector<float> pointRadiusSquaredDistance;
  	for (size_t i = 0; i < neighbours_.size(); ++i)
  	{
  		if ( tree->nearestKSearch (cloud->points[i], nnk, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
		{
			for (int j = 0; j < pointIdxRadiusSearch.size(); ++j)
				if (i != pointIdxRadiusSearch[j])
		    		neighbours_[i].insert(pointIdxRadiusSearch[j]);
		}
  	}

	computeQuadrics(cloud, normals, saliency);
	modified_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	*modified_cloud_ = *cloud;
	collapse(max_num_vertices);

	simplified_cloud->points.clear();
	std::map<int, int> valid_points;

	int counter = 0;
	for (int i = 0; i < point_status_.size(); ++i)
		if (point_status_[i])
		{
			valid_points[i] = counter++;
			simplified_cloud->points.push_back(modified_cloud_->points[i]);
		}
}

void SalientBasedSimplification::prepare(int num_points)
{
	collapsed_.clear();
	point_markers_.clear();
	point_status_.clear();
	point_markers_.resize(num_points, 0);
	point_status_.resize(num_points, true);

	collapsed_indices_.resize(num_points);
	for (unsigned int i = 0; i < collapsed_indices_.size(); ++i)
		collapsed_indices_[i].insert(i);
}

void SalientBasedSimplification::getClusters(
	std::vector<std::vector<int> > &clusters, 
	std::vector< std::vector<int> > &cluster_adjacency
)
{
	clusters.clear();
	cluster_adjacency.clear();

	std::map<int, int> index_map;

	int k  = 0;
	for (unsigned int i = 0; i < collapsed_indices_.size(); ++i)
	{
		if (point_status_[i])
		{
			index_map[i] = k;
			++k;
		}
	}

	clusters.resize(index_map.size());
	cluster_adjacency.resize(index_map.size());

	for (std::map<int, int>::iterator mit = index_map.begin(); mit != index_map.end(); ++mit)
	{
		int i = mit->first;
		int cluster_id = mit->second;

		std::copy(collapsed_indices_[i].begin(), collapsed_indices_[i].end(), std::back_inserter(clusters[cluster_id]));

		for (std::set<int>::iterator it = neighbours_[i].begin(); it != neighbours_[i].end(); ++it)
		{
			if (*it != i && point_status_[*it])
				cluster_adjacency[cluster_id].push_back(index_map[*it]);
		}
	}
}

void SalientBasedSimplification::computeQuadrics(
	const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
	const std::vector<pcl::Vertices> polygons,
	const std::vector<float> &saliency
) {
	point_quadrics_.clear();
	point_quadrics_.resize(cloud->points.size(), Eigen::Matrix4f::Zero());

	for (unsigned int i  = 0; i < polygons.size(); ++i) {
		const std::vector<uint32_t>& vertices = polygons[i].vertices;

		if (vertices.size() >= 3) {
			Eigen::Vector3f v0 = cloud->points[vertices[0]].getVector3fMap ();
			Eigen::Vector3f v1 = cloud->points[vertices[1]].getVector3fMap ();
			Eigen::Vector3f v2 = cloud->points[vertices[2]].getVector3fMap ();

			Eigen::Vector3f normal = (v1-v0).cross(v2-v1).normalized();
			float a = normal[0], b = normal[1], c = normal[2], d = - normal.dot(v0);
			Eigen::Matrix4f Q;
			Q << a*a, a*b, a*c, a*d,
				 a*b, b*b, b*c, b*d,
				 a*c, b*c, c*c, c*d,
				 a*d, b*d, c*d, d*d;

			for (int k = 0; k<vertices.size(); ++k)
				point_quadrics_[vertices[k]] += Q;
		}
	}

	if (saliency.size() == cloud->points.size())
		for (unsigned int i = 0; i < saliency.size(); ++i) {
			point_quadrics_[i] *= saliency[i];
		}
}

//http://lgg.epfl.ch/publications/2002/pauly_2002_ESP.pdf
void SalientBasedSimplification::computeQuadrics(
	const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
	const pcl::PointCloud<pcl::Normal>::Ptr &normals,
	const std::vector<float> &saliency
)
{
	point_quadrics_.clear();
	point_quadrics_.resize(cloud->points.size(), Eigen::Matrix4f::Zero());

	for (unsigned int i  = 0; i < neighbours_.size(); ++i)
	{
		std::vector<int> neighbourhood(neighbours_[i].begin(), neighbours_[i].end());

		for (unsigned int k = 0; k < neighbourhood.size(); ++k)
		{
			int j = neighbourhood[k];
			if (i >= j)	continue;

			
			Eigen::Vector3f ei = cloud->points[j].getVector3fMap () - cloud->points[i].getVector3fMap ();
			Eigen::Vector3f bi = ei.cross(normals->points[j].getNormalVector3fMap ());
			Eigen::Vector3f normal = (ei).cross(bi).normalized();
			float a = normal[0], b = normal[1], c = normal[2], d = - normal.dot(cloud->points[i].getVector3fMap ());

			Eigen::Matrix4f Q;
			Q << a*a, a*b, a*c, a*d,
				 a*b, b*b, b*c, b*d,
				 a*c, b*c, c*c, c*d,
				 a*d, b*d, c*d, d*d;

			point_quadrics_[i] += Q;
			point_quadrics_[j] += Q;
		}
	}

	if (saliency.size() == cloud->points.size())
		for (unsigned int i = 0; i < saliency.size(); ++i)
		{
			point_quadrics_[i] *= saliency[i];
		}
}

void SalientBasedSimplification::construct_pairs(int id)
{
	if (collapsed_.size() > 0)	point_markers_[id]++;
	for (std::set<int>::iterator it = neighbours_[id].begin(); it != neighbours_[id].end(); ++it)
	{
		if (*it == id)	continue;

		int other_id = *it;
		PointPair new_pair;

		if (other_id > id)
		{
			new_pair.id1 = other_id;
			new_pair.id2 = id;

			if (collapsed_.size() == 0)	continue;
		} else
		{
			new_pair.id2 = other_id;
			new_pair.id1 = id;
		}

		new_pair.marker1 = point_markers_[new_pair.id1];
		new_pair.marker2 = point_markers_[new_pair.id2];

		Eigen::Matrix4f Q = point_quadrics_[id] + point_quadrics_[other_id];
		Eigen::Matrix4f Q_mod = Q; Q_mod.row(3) = Eigen::RowVector4f(0, 0, 0, 1);
		Eigen::Matrix4f Q_mod_inv; bool invertible;
		Q_mod.computeInverseWithCheck (Q_mod_inv, invertible);
		/*if (invertible) {
			new_pair.target = Q_mod_inv*Eigen::Vector4f(0, 0, 0, 1);
			new_pair.cost = new_pair.target.transpose() * Q * new_pair.target;
		}
		else {
			Eigen::Vector4f v1 = modified_cloud_->points[new_pair.id1].getVector4fMap();
			Eigen::Vector4f v2 = modified_cloud_->points[new_pair.id2].getVector4fMap();
			Eigen::Vector4f mid = (v1+v2)/2.0;

			double c1= v1.transpose()*Q*v1;
			double c2 = v2.transpose()*Q*v2;
			double cmid = mid.transpose()*Q*mid;

		    if( c1 < c2 && c1 < cmid) { new_pair.cost = c1; new_pair.target = v1; }
		    else if (c2 < c1 && c2 < cmid)	{ new_pair.cost = c2; new_pair.target = v2; }
		    else { new_pair.cost = cmid; new_pair.target = mid; }
		} */
			Eigen::Vector4f v1 = modified_cloud_->points[new_pair.id1].getVector4fMap();
			Eigen::Vector4f v2 = modified_cloud_->points[new_pair.id2].getVector4fMap();
			Eigen::Vector4f mid = (v1+v2)/2.0;
			double c1= v1.transpose()*Q*v1;
			double c2 = v2.transpose()*Q*v2;
			double cmid = mid.transpose()*Q*mid;
			new_pair.cost = c2; new_pair.target = v2;
			// new_pair.cost = cmid; new_pair.target = mid;

		point_pairs_.push(new_pair);
	}
}

void SalientBasedSimplification::collapse(
	int max_num_vertices)
{
	prepare(modified_cloud_->points.size());
	for (int i = 0; i < modified_cloud_->points.size(); ++i) 
		construct_pairs(i);

	while (!point_pairs_.empty() && modified_cloud_->points.size() - collapsed_.size() > max_num_vertices)
	{
		PointPair cur = point_pairs_.top();
		point_pairs_.pop();

		if (!point_status_[cur.id1] || !point_status_[cur.id2] || cur.marker1 != point_markers_[cur.id1] || cur.marker2 != point_markers_[cur.id2])
		{
			// std::cout << "Skipped: " << cur.id1 << " " << cur.id2 << "\n";
			continue;
		}
		// else std::cout << collapsed_.size() << " collapse: " << cur.id1 << " " << cur.id2 << " " << cur.marker1 << " " << cur.marker2 << " " << cur.cost << "\n";

		collapsed_.push_back(cur);

		neighbours_[cur.id2].erase(cur.id1);
		neighbours_[cur.id1].erase(cur.id2);
		for (std::set<int>::iterator it = neighbours_[cur.id1].begin(); it != neighbours_[cur.id1].end(); ++it)
		{
			int neigh_id = *it;
			neighbours_[neigh_id].erase(cur.id1);
			
			/*std::set<int>::iterator it2 = neighbours_[neigh_id].find(cur.id1);
			if (it2 != neighbours_[neigh_id].end())
			{
				neighbours_[neigh_id].erase(cur.id1);
				neighbours_[cur.id1].erase(neigh_id);
				std::cout << "Degenerate edge: " <<
			} else*/
			{
				neighbours_[neigh_id].insert(cur.id2);
				neighbours_[cur.id2].insert(neigh_id);
			}
		}
		neighbours_[cur.id1].clear();
		point_status_[cur.id1] = false;

		collapsed_indices_[cur.id2].insert(collapsed_indices_[cur.id1].begin(),collapsed_indices_[cur.id1].end());

		point_quadrics_[cur.id2] += point_quadrics_[cur.id1];
		modified_cloud_->points[cur.id2].x = cur.target[0];
		modified_cloud_->points[cur.id2].y = cur.target[1];
		modified_cloud_->points[cur.id2].z = cur.target[2];

		construct_pairs(cur.id2);
	}

	/*int test;
	std::cin >> test;*/
}