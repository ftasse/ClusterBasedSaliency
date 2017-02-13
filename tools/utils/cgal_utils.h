#ifndef CGAL_UTILS_H
#define CGAL_UTILS_H

#include <vector>
#include <Eigen/Core>

void computeAndOrientNormals(
	const std::vector<Eigen::Vector3f> &points,
	std::vector<Eigen::Vector3f> &normals,
	size_t n_neighbours,
	bool only_orient);

#endif // CGAL_UTILS_H