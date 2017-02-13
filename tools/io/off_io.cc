#include "io/off_io.h"

#include <fstream>
#include <pcl/conversions.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PolygonMesh.h>

namespace pcl {
  namespace io {
    PCL_EXPORTS int loadPolygonFileOFF(const std::string &file_name, 
				       pcl::PolygonMesh& mesh) {
      std::ifstream ifs(file_name.c_str());
      size_t num_vertices = 0, num_faces = 0, num_edges = 0;
      std::string header;

      ifs >> header;
      if (header != "OFF") {
	std::cout << "Failed to load mesh from a file that is not an OFF file: "
		  << file_name << "\n";
	ifs.close();
	return false;
      }

      ifs >> num_vertices >> num_faces >> num_edges;
      if (num_vertices <= 0) {
	std::cout << "The number of mesh vertices in this file is not positive: "
		  << file_name << "\n";
	ifs.close();
	return false;
      }

      mesh.polygons.clear ();
      mesh.cloud.data.clear ();
      mesh.cloud.width = mesh.cloud.height = 0;
      mesh.cloud.is_dense = true;

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (
        new pcl::PointCloud<pcl::PointXYZ> ());
      cloud->points.resize (num_vertices);
      cloud->width = static_cast<uint32_t> (cloud->points.size ());
      cloud->height = 1;
      cloud->is_dense = true; 

      for (size_t i = 0; i < num_vertices; ++i) {
	ifs >> cloud->points[i].x 
	    >> cloud->points[i].y
	    >> cloud->points[i].z;
      }

      mesh.polygons.resize (num_faces);
      for (size_t i = 0; i < num_faces; ++i) {
	size_t num_face_vertices = 0;
	ifs >> num_face_vertices;
	mesh.polygons[i].vertices.resize (num_face_vertices);
	for (size_t j = 0; j < num_face_vertices; ++j) {
	  ifs >> mesh.polygons[i].vertices[j];
	}
      }

      pcl::toPCLPointCloud2(*cloud, mesh.cloud);
      return true;
    }

    PCL_EXPORTS int loadPolygonFile(const std::string &file_name,
           pcl::PolygonMesh& mesh) {
      std::string extension = file_name.substr (file_name.find_last_of (".") + 1);

      if (extension == "off") {
        return loadPolygonFileOFF(file_name, mesh);
      }
      else if (extension == "pcd") // no Polygon, but only a point cloud
      {
        pcl::io::loadPCDFile (file_name, mesh.cloud);
        mesh.polygons.resize (0);
        return (static_cast<int> (mesh.cloud.width * mesh.cloud.height));
      }
      // else if (extension == "vtk")
      //  return (pcl::io::loadVTKFile (file_name, mesh));
      else if (extension == "ply")
       return (pcl::io::loadPLYFile (file_name, mesh));
      else if (extension == "obj")
        return (pcl::io::loadOBJFile (file_name, mesh));
      // else if (extension == "stl" )
      //   return (pcl::io::loadPolygonFileSTL (file_name, mesh));
      else
      {
        // return loadPolygonFile(file_name, mesh);
        PCL_ERROR ("[pcl::io::loadPolygonFile]: Unsupported file type (%s)\n", extension.c_str ());
        return (0);
      }
    }

  } // io
} // pcl
