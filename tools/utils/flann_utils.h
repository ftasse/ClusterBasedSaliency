#ifndef FLANN_UTILS_H
#define FLANN_UTILS_H

#include <flann/flann.h>

void nearestKSearch (flann::Index<flann::ChiSquareDistance<float> > &index, const float* histogram, 
                	 int k, std::vector< std::vector<int> > &indices, std::vector< std::vector<float> > &distances)
{
  // Query point
  int dim = sizeof(histogram)/sizeof(float);
  flann::Matrix<float> p = flann::Matrix<float>(new float[dim], 1, dim);
  memcpy (&p.ptr ()[0], histogram, p.cols * p.rows * sizeof (float));

  index.knnSearch (p, indices, distances, k, flann::AutotunedIndexParams(0.99));
  delete[] p.ptr ();
}

void radiusSearch (flann::Index<flann::ChiSquareDistance<float> > &index, const float* histogram, 
                	 float radius, std::vector< std::vector<int> > &indices, std::vector< std::vector<float> > &distances)
{
  // Query point
  int dim = sizeof(histogram)/sizeof(float);
  flann::Matrix<float> p = flann::Matrix<float>(new float[dim], 1, dim);
  memcpy (&p.ptr ()[0], histogram, p.cols * p.rows * sizeof (float));

  index.radiusSearch (p, indices, distances, radius, flann::AutotunedIndexParams(0.99));
  delete[] p.ptr ();
}

#endif // FLANN_UTILS_H