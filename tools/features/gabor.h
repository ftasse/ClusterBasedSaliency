#ifndef GABOR_H_
#define GABOR_H_

#include <pcl/filters/convolution_3d.h>

namespace pcl {
  namespace filters
  {
    void createGaborFilterBank(int u, int v, int m, int n, std::vector<Eigen::MatrixXf>& bank);

    template<typename PointInT, typename PointOutT>
    class GaborKernel : public ConvolvingKernel <PointInT, PointOutT>
    {
      public:
        using ConvolvingKernel<PointInT, PointOutT>::initCompute;
        using ConvolvingKernel<PointInT, PointOutT>::input_;
        using ConvolvingKernel<PointInT, PointOutT>::operator ();
        using ConvolvingKernel<PointInT, PointOutT>::makeInfinite;
        typedef boost::shared_ptr<GaborKernel<PointInT, PointOutT> > Ptr;
        typedef boost::shared_ptr<GaborKernel<PointInT, PointOutT> > ConstPtr;

        /** Default constructor */
        GaborKernel ()
          : ConvolvingKernel <PointInT, PointOutT> ()
          , sigma_ (0)
          , threshold_ (std::numeric_limits<float>::infinity ())
        {}

        virtual ~GaborKernel () {}

        /** Set the sigma parameter of the Gaussian
          * \param[in] sigma
          */
        inline void
        setSigma (float sigma) { sigma_ = sigma; }

        /** Set the distance threshold relative to a sigma factor i.e. points such as
          * ||pi - q|| > sigma_coefficient^2 * sigma^2 are not considered.
          */
        inline void
        setThresholdRelativeToSigma (float sigma_coefficient)
        {
          sigma_coefficient_.reset (sigma_coefficient);
        }

        /** Set the distance threshold such as pi, ||pi - q|| > threshold are not considered. */
        inline void
        setThreshold (float threshold) { threshold_ = threshold; }

        /** Must call this methode before doing any computation */
        bool initCompute ();

        virtual PointOutT
        operator() (const std::vector<int>& indices, const std::vector<float>& distances);

      protected:
        float sigma_;
        float sigma_sqr_;
        float threshold_;
        boost::optional<float> sigma_coefficient_;
    };
  }
}

#endif // GABOR_H_