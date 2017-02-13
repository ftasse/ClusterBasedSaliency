#include <stdio.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/time.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/search/kdtree.h>

#include "tools/utils/cmd_parser.h"
#include "tools/features/normals.h"
#include "tools/io/off_io.h"
#include "tools/utils/pcl_utils.h"
#include "tools/utils/shading_utils.h"
#include "segmentation/segment.h"
#include "segmentation/segmentation_utils.h"

int main(int argc, char **argv) {
  // Parse cmd flags
  std::map<std::string, std::string> flags;
  parseCmdArgs(argc, argv, &flags);

  std::string model_filename, output_filename;
  if (!getFlag(flags, "model", "",  &model_filename)) {
    printf("Error: specify --model\n");
    return 1;
  }
  if (!getFlag(flags, "output", "",  &output_filename)) {
    printf("Error: specify --output\n");
    return 1;
  }
  int num_segments;
  float spatial_importance, normal_importance, fpfh_importance;
  float neighbourhood_radius_percent;
  getFlag(flags, "num_segments", 100, &num_segments);
  getFlag(flags, "spatial_importance", 1.0f, &spatial_importance);
  getFlag(flags, "normal_importance", 1.0f, &normal_importance);
  getFlag(flags, "fpfh_importance", 1.0f, &fpfh_importance);
  getFlag(flags, "radius_percent", 0.03f, &neighbourhood_radius_percent);
  
  // Load model
  pcl::PolygonMesh mesh;
  if (!pcl::io::loadPolygonFile(model_filename, mesh)) {
    printf("Error: could not load model from %s.\n", model_filename.c_str());
    return 1;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr points (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33>);
  
  pcl::fromPCLPointCloud2(mesh.cloud, *points);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (points);

  float support_radius = neighbourhood_radius_percent/getNormalizingScaleFactor(points, false);
  printf("computed support radius: %f\n", support_radius);

  computeOrientedNormalsAndFPFHS(points, normals, fpfhs, 
                         support_radius, support_radius*1.25, tree);
  printf("computed normals and descriptors.\n");

  std::vector<Segment> segments;
  float voxel_resolution = support_radius;
  pcl::StopWatch watch;
  computeSuperClustersPapon(points, normals, fpfhs,
                            segments, num_segments, 
                            voxel_resolution, 0, support_radius, false,
                            spatial_importance, normal_importance,
                            fpfh_importance);
  std::cout << "Elapsed segmentation time: " << watch.getTimeSeconds() << "\n";

  /*std::vector<std::vector<int> > segment_adjacency;
  std::vector<std::vector<size_t> > neighbours;
  std::vector<std::vector<float> > neighbours_sqrdist;
  computeNeighbours(points, neighbours, neighbours_sqrdist, 18);
  pcl::computeSegmentAdjacencyWithNeighbourhood(
    segments, neighbours, neighbours_sqrdist, segment_adjacency);*/
  
  printf("num of faces: %zd. num of vertices: %zd. %zd segments\n",
         mesh.polygons.size(), normals->size(), segments.size());

  pcl::PointCloud<pcl::RGB>::Ptr colors (new pcl::PointCloud<pcl::RGB>);
  // convertNormalsToRGBs(normals, colors);
  convertSegmentsToRGBs(segments, colors, points->size());
  // convertSegmentsBoundariesToBlack(points, segments, colors, points->size());
  concatenatePointsNormalsRGBs(points, normals, colors, mesh.cloud);  
  pcl::io::savePolygonFile(output_filename.c_str(), mesh);
  return 0;
}
