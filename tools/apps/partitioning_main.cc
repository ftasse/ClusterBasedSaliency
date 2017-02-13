#include <stdio.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/centroid.h>
#include <pcl/common/time.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/search/kdtree.h>

#include "utils/cmd_parser.h"
#include "features/normals.h"
#include "io/off_io.h"
#include "utils/pcl_utils.h"
#include "utils/shading_utils.h"
// #include "segmentation/segment.h"
// #include "segmentation/segmentation_utils.h"
#include "mincut_partitioning/segmentation.h"

int main(int argc, char **argv) {
  // Parse cmd flags
  std::map<std::string, std::string> flags;
  parseCmdArgs(argc, argv, &flags);

  std::string model_filename, output_filename, scalar_function_filename;
  int num_mincut_partitions;
  float neighbourhood_radius_percent;
  bool use_topology = false;
  bool recursive = true, hierarchical = false;
  float lambda = 2, alpha = 0.8;
  if (!getFlag(flags, "model", "",  &model_filename)) {
    printf("Error: specify --model\n");
    return 1;
  }
  if (!getFlag(flags, "output", "",  &output_filename)) {
    printf("Error: specify --output\n");
    return 1;
  }
  getFlag(flags, "scalar_function", "",  &scalar_function_filename);
  getFlag(flags, "radius_percent", 0.03f, &neighbourhood_radius_percent);
  getFlag(flags, "num_mincut_partitions", 2, &num_mincut_partitions);
  getFlag(flags, "use_topology", false, &use_topology);
  getFlag(flags, "recursive", true, &recursive);
  getFlag(flags, "hierarchical", false, &hierarchical);
  getFlag(flags, "lambda", std::max(1.0f, 0.5f*num_mincut_partitions), &lambda); //
  getFlag(flags, "alpha", 0.5f, &alpha);

  // Load model
  pcl::PolygonMesh mesh;
  if (!pcl::io::loadPolygonFile(model_filename, mesh)) {
    printf("Error: could not load model from %s.\n", model_filename.c_str());
    return 1;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr points (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::fromPCLPointCloud2(mesh.cloud, *points);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (points);

  float support_radius = neighbourhood_radius_percent/getNormalizingScaleFactor(points, false);
  size_t nb_neighbours = std::max(9,std::min(18, pcl::computeAverageDensity(points, support_radius, tree)));
  printf("computed support radius: %f %zd\n", support_radius, nb_neighbours);

  computeNormals(points, normals, support_radius, tree);
  computeAndOrientNormalsWithCGAL(points, normals, nb_neighbours, true);
  printf("computed normals.\n");


  std::vector<float> saliency;
  if (!scalar_function_filename.empty()) 
      loadScalarFunction(scalar_function_filename.c_str(), saliency);

  if (saliency.size() != points->size()) {
    std::cout << "Use default saliency. " << saliency.size() << "\n";
    saliency.clear();
    Eigen::Vector4f centroid; 
    pcl::compute3DCentroid(*points,centroid); 
    saliency.resize(points->size(), 1.0);
    for (size_t i = 0; i < points->size(); ++i) {
      saliency[i] = (points->points[i].getVector3fMap() - Eigen::Vector3f(centroid[0], centroid[1], centroid[2])).norm();
    }
  }

  pcl::StopWatch watch;
  std::vector<std::vector<size_t> > neighbours(points->size());
  std::vector<std::vector<float> > neighbours_weights(points->size());
  if (!use_topology || mesh.polygons.size() == 0) {
    for (size_t i = 0; i < points->size(); ++i) {
      std::vector<int> nn_indices;
      tree->radiusSearch(i, support_radius, nn_indices, neighbours_weights[i]);
      for (size_t k = 0; k < nn_indices.size(); ++k) neighbours[i].push_back(nn_indices[k]);
    }
  } else {
    for (int i = 0; i < mesh.polygons.size(); ++i) {
      pcl::Vertices polygon = mesh.polygons[i];
      for (int k = 0; k < polygon.vertices.size(); ++k) {
        int idx1 = polygon.vertices[k];
        int idx2 = polygon.vertices[(k+1)%polygon.vertices.size()];
        neighbours[idx1].push_back(idx2);
        neighbours_weights[idx1].push_back(0.0f);
      }
    }
  }

  // compute edge weights
  // https://users.cs.cf.ac.uk/Yukun.Lai/papers/cagd09.pdf
  std::set<int> convex_edges;
  int num_convex_edges = 0, num_concave_edges = 0;
  for (size_t i = 0; i < neighbours.size(); ++i) {
    // neighbours[i].erase(neighbours[i].begin());
    for (size_t k = 0; k < neighbours[i].size(); ++k) {
      if (i == neighbours[i][k])  continue;
      Eigen::Vector3f n1 = normals->points[i].getNormalVector3fMap();
      Eigen::Vector3f n2 = normals->points[neighbours[i][k]].getNormalVector3fMap();
      Eigen::Vector3f edge = points->points[i].getVector3fMap() - 
          points->points[neighbours[i][k]].getVector3fMap();
      float cosine = n1.dot(n2);
      float sin_sign = n1.cross(n2).dot(edge);
      double convexFac = 1;
      if (sin_sign > 1e-2) {  // is a convex edge. boundaries should contain these
        convexFac = 0.2;
        convex_edges.insert(i);
        convex_edges.insert(neighbours[i][k]);
        num_convex_edges++;
      } else {
        // convexFac = 1;
        num_concave_edges++;
      }

      // double weight = std::abs(acos(cosine)/M_PI)/convexFac;
      double weight = (1-cosine)*convexFac;
      neighbours_weights[i][k] = weight;
    }
  }
  std::cout << " num convex/concave: " << num_convex_edges << " " << num_concave_edges << "\n";

  std::vector< std::set<int> > labelling;
  // labelling.resize(2);
  // labelling[0] = convex_edges;
  // for (size_t i = 0; i < points->size(); ++i)
  //   if (convex_edges.find(i) == convex_edges.end())  labelling[1].insert(i);
  segmentation(
    points, neighbours, neighbours_weights,
    Eigen::Map<Eigen::VectorXf>(&saliency[0], points->size()),
    labelling, num_mincut_partitions, recursive, hierarchical, lambda, alpha);
  std::cout << "Total num labels: " << labelling.size() << "\n";

  pcl::console::print_highlight ("Computed mincut partioning. Time: %f\n", watch.getTimeSeconds());
  std::vector<int> point_labels(points->size(), -1);
  std::vector<std::vector<int> > tmp_segments(labelling.size());
  for (size_t i = 0; i < labelling.size(); ++i) {
    std::copy(labelling[i].begin(), labelling[i].end(), 
              std::back_inserter(tmp_segments[i]));
    for (size_t k = 0; k < tmp_segments[i].size(); ++k)
      point_labels[tmp_segments[i][k]] = i;
  }

  std::cout << "Elapsed segmentation time: " << watch.getTimeSeconds() << "\n";

  printf("num of faces: %zd. num of vertices: %zd. %zd segments\n",
         mesh.polygons.size(), normals->size(), tmp_segments.size());

  // convertNormalsToRGBs(normals, colors);
  pcl::PointCloud<pcl::RGB>::Ptr colors (new pcl::PointCloud<pcl::RGB>);
  colors->points.clear();
  colors->width = points->size();
  colors->height = 1;
  colors->points.resize(points->size());
  
  for (size_t s = 0; s < tmp_segments.size(); ++s) {
    pcl::RGB color;
    color.r = rand()%255;
    color.b = rand()%255;
    color.g = rand()%255;
    for (size_t k = 0; k < tmp_segments[s].size(); ++k) {
      colors->points[tmp_segments[s][k]] = color;
    }
  }
  // convertSegmentsBoundariesToBlack(points, segments, colors, points->size());
  concatenatePointsNormalsRGBs(points, normals, colors, mesh.cloud);  
  pcl::io::savePolygonFile(output_filename.c_str(), mesh);
  return 0;
}
