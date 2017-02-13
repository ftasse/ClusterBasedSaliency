#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include <string>
#include <vector>
#include <set>

#include <Eigen/Core>

#include <pcl/point_cloud.h>


namespace pcl {
  class PointXYZ;
}

/*struct GMMFit {
    Eigen::MatrixXf* samples;
    int num_labels;
    bool use_faces;
    std::vector< int > labels;
    std::vector< std::vector<double> > probs;

    GMMFit(std::vector<double>& weights, int n_labels, bool _use_faces = true);
    GMMFit(std::vector< std::vector<double> >& weights, int n_labels, bool _use_faces = true);
    ~GMMFit();

    void train(std::vector<int> mask_ids);
};*/

void segmentation(pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
                  const std::vector<std::vector<size_t>  >& neighbours,
                  const std::vector<std::vector<float> >& neighbours_weights,
                  const Eigen::MatrixXf& samples,
                  std::vector< std::set<int> >& labelling, int num_labels,
                  bool recursive = false, bool hierarchical = false,
                  double lambda = 25, double alpha = 0.5);

#endif //SEGMENTATION_H
