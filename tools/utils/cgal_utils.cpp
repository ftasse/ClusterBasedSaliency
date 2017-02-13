#ifdef CGAL_LIB_EXISTS
//#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Cartesian.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
// #include <CGAL/jet_estimate_normals.h>

// Types
typedef  CGAL::Cartesian<float> Kernel;
// typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;
// Point with normal vector stored in a std::pair.
#endif

#include "utils/cgal_utils.h"
#include <iostream>
#include <Eigen/Core>

void computeAndOrientNormals(
    const std::vector<Eigen::Vector3f> &points,
    std::vector<Eigen::Vector3f> &normals,
    size_t n_neighbours,
    bool only_orient) {

#ifdef CGAL_LIB_EXISTS
    typedef boost::tuple<int, Point, Vector> IndexedPointWithNormalTuple;
	std::list<IndexedPointWithNormalTuple> tpoints;

    for (unsigned int i = 0; i < points.size(); ++i)
    {
        Point cgal_point(points[i][0], points[i][1], points[i][2]);
        Vector normal;
        if (normals.size() == points.size()) 
            normal = Vector(normals[i][0], normals[i][1], normals[i][2]);
        
        tpoints.push_back(IndexedPointWithNormalTuple(i, cgal_point, normal));
    }

    if (!(normals.size() == points.size() && only_orient)) {
    	// Estimates normals direction.
        // Note: pca_estimate_normals() requires an iterator over points
        // as well as property maps to access each point's position and normal.
        CGAL::pca_estimate_normals(tpoints.begin(), tpoints.end(),
                                   CGAL::Nth_of_tuple_property_map<1,IndexedPointWithNormalTuple>(),
                                   CGAL::Nth_of_tuple_property_map<2,IndexedPointWithNormalTuple>(),
                                   n_neighbours);
    }

    normals.resize(points.size());
    for (std::list<IndexedPointWithNormalTuple>::iterator it = tpoints.begin(); it != tpoints.end(); ++it) {
        int  i = it->get<0>();
        Vector cgal_normal = it->get<2>();
        normals[i] = Eigen::Vector3f(cgal_normal[0], cgal_normal[1], cgal_normal[2]);
    }

    // Orients normals.
    // Note: mst_orient_normals() requires an iterator over points
    // as well as property maps to access each point's position and normal.
    std::list<IndexedPointWithNormalTuple>::iterator unoriented_points_begin =
        CGAL::mst_orient_normals(tpoints.begin(), tpoints.end(),
                                CGAL::Nth_of_tuple_property_map<1,IndexedPointWithNormalTuple>(),
                                CGAL::Nth_of_tuple_property_map<2,IndexedPointWithNormalTuple>(),
                                n_neighbours);
    tpoints.erase(unoriented_points_begin, tpoints.end());

    normals.resize(points.size());
    for (std::list<IndexedPointWithNormalTuple>::iterator it = tpoints.begin(); it != tpoints.end(); ++it) {
        int  i = it->get<0>();
        Vector cgal_normal = it->get<2>();
        normals[i] = Eigen::Vector3f(cgal_normal[0], cgal_normal[1], cgal_normal[2]);
    }

#else
    std::cout << "Error: CGAL library does not exists. This operation cannot run without it. !\n";
#endif
}

