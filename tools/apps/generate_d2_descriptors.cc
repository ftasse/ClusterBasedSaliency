#include <float.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <algorithm>

// #include <omp.h>
#include <pcl/common/time.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>

#include "utils/cmd_parser.h"
#include "utils/matrix_io.h"
#include "io/off_io.h"
/*#include "tools/utils/eigen3_utils.h"
#include "tools/utils/pcl_utils.h"
#include "tools/utils/shading_utils.h"*/

#define kNumPairDistances 1024*1024
#define kNumHistogramBins 128

void computeHistogram(Eigen::RowVectorXf &values, Eigen::RowVectorXf &hists, int num_bins)
{
  float min_val = values.minCoeff();
  float max_val = values.maxCoeff();

  hists.setZero(num_bins);

  for (int i = 0; i < values.size(); ++i)
  {
    float tmp = (max_val - values[i])/(max_val - min_val);
    int hidx = tmp * (num_bins-1);
    hists[hidx]++;
  }

  hists /= hists.maxCoeff();
}

void generate_d2_descriptors(
  pcl::PolygonMesh& mesh, size_t nb_point_pairs, size_t num_bins, 
  Eigen::RowVectorXf& descriptor) 
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr points (
    new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(mesh.cloud, *points);

  srand(time(NULL));
  Eigen::RowVectorXf random_pair_distances(nb_point_pairs);
  for (unsigned int i = 0; i < random_pair_distances.size(); ++i)
  {
    Eigen::Vector3f point1 (points->points[rand()%points->points.size()].getArray3fMap());
    Eigen::Vector3f point2 (points->points[rand()%points->points.size()].getArray3fMap());
    random_pair_distances[i] = (point1-point2).squaredNorm();
  }
  computeHistogram(random_pair_distances, descriptor, num_bins);
}

int main(int argc, char** argv) {
  pcl::StopWatch watch;

  // Parse cmd flags
  std::map<std::string, std::string> flags;
  parseCmdArgs(argc, argv, &flags);
  bool arg_error = false;

  std::string model_filename, model_filelist;
  std::string output_prefix;
  size_t num_bins, nb_point_pairs;
  if (!(getFlag(flags, "model", "",  &model_filename) ^
        getFlag(flags, "model_filelist", "",  &model_filelist))) {
    printf("Error: specify --model xor --model_filelist\n");
    arg_error = true;
  }
  if (!getFlag(flags, "output_prefix", "",  &output_prefix)) {
    printf("Error: specify --output_prefix\n");
    arg_error = true;
  }
  getFlag(flags, "num_bins", (size_t) kNumHistogramBins,  &num_bins);
  getFlag(flags, "num_point_pairs", (size_t) kNumPairDistances, &nb_point_pairs);

  bool verbose; getFlag(flags, "verbose", false, &verbose);
  if (arg_error) return 1;

  std::string output_filename = output_prefix + "_d2_descriptors.txt";

  if (model_filename.size() > 0) {
    pcl::PolygonMesh mesh;
    if (!pcl::io::loadPolygonFile(model_filename, mesh)) {
      printf("Error: could not load model from %s.\n", model_filename.c_str());
      return 1;
    }
    Eigen::RowVectorXf descriptor;
    generate_d2_descriptors(mesh, nb_point_pairs, num_bins, descriptor);
    saveEigenMatrix(output_filename, descriptor);
  }

  if  (model_filelist.size() > 0) {
    std::ifstream model_filelist_in (model_filelist.c_str());

    int counter = 0;
    while (!model_filelist_in.eof()) {
      std::string model_id, filename;
      model_filelist_in >> model_id >> filename;
      if (model_id.size() == 0 && filename.size() == 0) continue;

      if (verbose)
        std::cout << "Processing mesh: " << filename << "\n";

      pcl::PolygonMesh mesh;
      if (!pcl::io::loadPolygonFile(filename, mesh)) {
        printf("Error: could not load model from %s.\n", filename.c_str());
        continue;
      }
      Eigen::RowVectorXf descriptor;
      generate_d2_descriptors(mesh, nb_point_pairs, num_bins, descriptor);
      saveEigenMatrix(output_filename, descriptor, counter > 0);
      ++counter;
    }

    model_filelist_in.close();
  }

  return 0;
}