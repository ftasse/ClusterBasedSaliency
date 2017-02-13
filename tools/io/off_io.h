#ifndef TOOLS_IO_OFFIO_H
#define TOOLS_IO_OFFIO_H

#include <string>

namespace pcl {
  class PolygonMesh;

  namespace io {
    int loadPolygonFileOFF(const std::string& file_name, 
			   pcl::PolygonMesh& mesh);

    int loadPolygonFile(const std::string &file_name,
			     pcl::PolygonMesh& mesh);
  } // io
} // pcl

#endif // TOOLS_IO_OFFIO_H
