#ifndef EIGEN3_UTILS_H
#define EIGEN3_UTILS_H

#include <vector>
#include <Eigen/Core>

void normalizeFloatVector(std::vector<float> &values);

void logNormalizeFloatVector(std::vector<float> &values, float a = 4);

void removeOutliers(std::vector<float> &values, 
                    float min_percent, 
                    float max_percent);

void reduceDimension(Eigen::MatrixXf &data, int new_dim = 0);

void normalizeData(Eigen::MatrixXf &data);

float compute_variation(const Eigen::MatrixXf &data_points, Eigen::RowVectorXf mean);

Eigen::MatrixXf distL1(const Eigen::MatrixXf &x, const Eigen::MatrixXf &y);

Eigen::MatrixXf distEucSq(const Eigen::MatrixXf &x, const Eigen::MatrixXf &y);

Eigen::MatrixXf distCosine(const Eigen::MatrixXf &x, const Eigen::MatrixXf &y);


Eigen::MatrixXf distEmd(const Eigen::MatrixXf &x, const Eigen::MatrixXf &y);

Eigen::MatrixXf distChiSq(const Eigen::MatrixXf &x, const Eigen::MatrixXf &y);

#endif // EIGEN3_UTILS_H

