#include "features/gabor.h"

namespace pcl{
namespace filters {

// http://stackoverflow.com/questions/24574877/how-to-implement-gabor-filters-on-3d-images-in-matlab
// http://en.wikipedia.org/wiki/Gabor_filter
// 3D Gabor filter: http://www.sciencedirect.com/science/article/pii/S0031320312003421
// void createGaborFilterBank(int u, int v, int m, int n, std::vector<Eigen::MatrixXf>& gabor_bank) {
//   gabor_bank.resize(u*v);
//   float fmax = 0.25;
//   float gama = sqrt(2);
//   float eta = sqrt(2);
  
//   for (int i = 0; i < u; ++i) {
//     float fu = fmax/pow(sqrt(2), i);
//     float alpha = fu/gama;
//     float beta = fu/eta;

//     for (int j = 0; j < v; ++j) {
//     float tetav = (j/v)*M_PI;
//     Eigen::MatriXf filter = Eigen::MatrixXf::Zero(m, n);

//       for (int x = 0; x < m; ++x) {
//           for (int y = 0; y < n; ++y) {
//             float xprime = (x-((m+1)/2))*cos(tetav)+(y-((n+1)/2))*sin(tetav);
//             float yprime = -(x-((m+1)/2))*sin(tetav)+(y-((n+1)/2))*cos(tetav);
//             filter(x,y) = (fu*fu/(pi*gama*eta))*exp(-((alpha*alpha)*(xprime*xprime)+(beta*beta)*(yprime*yprime)))*exp(1i*2*M_PI*fu*xprime);
//           }
//         }
//       }
//       gabor_bank[i*v + j] = filter;
//     }
//   }
// }

template<typename PointInT, typename PointOutT>
bool GaborKernel<PointInT, PointOutT>::initCompute () {
  if (sigma_ == 0) {
    PCL_ERROR ("Sigma (%f) is not set or equal to 0!\n", sigma_);
    return (false);
  }
  sigma_sqr_ = sigma_ * sigma_;

  if (sigma_coefficient_) {
    if ((*sigma_coefficient_) > 6 || (*sigma_coefficient_) < 3)
    {
      PCL_ERROR ("Sigma coefficient (%f) out of [3..6]!\n", (*sigma_coefficient_));
      return (false);
    }
    else
      threshold_ = (*sigma_coefficient_) * (*sigma_coefficient_) * sigma_sqr_;
  }

  return (true);
}

template<typename PointInT, typename PointOutT>
PointOutT GaborKernel<PointInT, PointOutT>::operator() (const std::vector<int>& indices, const std::vector<float>& distances) {
  using namespace pcl::common;
  PointOutT result;
  float total_weight = 0;
  std::vector<float>::const_iterator dist_it = distances.begin ();

  for (std::vector<int>::const_iterator idx_it = indices.begin ();
       idx_it != indices.end ();
       ++idx_it, ++dist_it) {
    if (*dist_it <= threshold_ && isFinite ((*input_) [*idx_it])) {
      float weight = expf (-0.5f * (*dist_it) / sigma_sqr_);
      result += weight * (*input_) [*idx_it];
      total_weight += weight;
    }
  }
  if (total_weight != 0)
    result /= total_weight;
  else
    makeInfinite (result);

  return (result);
}

// template <>
// pcl::PointXYZL GaborKernel<pcl::PointXYZL,pcl::PointXYZL >::operator() (const std::vector<int>& indices, const std::vector<float>& distances)  {
//   using namespace pcl::common;
//   pcl::PointXYZL result;
//   float total_weight = 0;
//   std::vector<float>::const_iterator dist_it = distances.begin ();

//   for (std::vector<int>::const_iterator idx_it = indices.begin ();
//        idx_it != indices.end ();
//        ++idx_it, ++dist_it) {
//     if (*dist_it <= threshold_ && isFinite ((*input_) [*idx_it])) {
//       float weight = expf (-0.5f * (*dist_it) / sigma_sqr_);
//       result += weight * (*input_) [*idx_it];
//       total_weight += weight;
//     }
//   }
//   if (total_weight != 0)
//     result /= total_weight;
//   else
//     makeInfinite (result);

//   return (result);
// }

template class GaborKernel<pcl::PointXYZ, pcl::PointXYZ>;
template class GaborKernel<pcl::PointXYZI, pcl::PointXYZI>;
template class GaborKernel<pcl::PointXYZINormal, pcl::PointXYZINormal>;


} // namespace filters
} // namespace pcl