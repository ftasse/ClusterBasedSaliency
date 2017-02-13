#ifndef DIST_UTILS_H
#define DIST_UTILS_H

#include <flann/flann.h>

namespace flann {
  template<class T>
  struct CosineDistance
  {
      // typedef bool is_kdtree_distance;

      typedef T ElementType;
      typedef typename Accumulator<T>::Type ResultType;

      /**
       *  Compute the cosine angle distance
       */
      template <typename Iterator1, typename Iterator2>
      ResultType operator()(Iterator1 a, Iterator2 b, size_t size, ResultType worst_dist = -1) const
      {
        ResultType result = ResultType();
        ResultType diff0, diff1, diff2, diff3;
        Iterator1 last = a + size;
        Iterator1 lastgroup = last - 3;

        ResultType a_sum = ResultType(), b_sum = ResultType();
        Iterator1 a2 = a;
        Iterator2 b2 = b;
        for (int i = 0; i < size; ++i) {
          a_sum += *a2 * *a2; a2++;
          b_sum += *b2 * *b2; b2++;
        }
        ResultType denom = sqrt(a_sum)*sqrt(b_sum);

        // Add one to prevent negative distance.
        result += 1;

        /* Process 4 items with each loop for efficiency. */
        while (a < lastgroup) {
            diff0 = (ResultType)(a[0] * b[0])/denom;
            diff1 = (ResultType)(a[1] * b[1])/denom;
            diff2 = (ResultType)(a[2] * b[2])/denom;
            diff3 = (ResultType)(a[3] * b[3])/denom;
            result -= (diff0 + diff1 + diff2 + diff3);
            a += 4;
            b += 4;

            if ((worst_dist>0)&&(result>worst_dist)) {
                return result;
            }
        }
        /* Process last 0-3 pixels.  Not needed for standard vector lengths. */
        while (a < last) {
            diff0 = (ResultType)((*a++) * (*b++))/denom;
            result -= diff0;
        }
        return result;
      }

      /**
       * Partial distance, used by the kd-tree.
       */
      // template <typename U, typename V>
      // inline ResultType accum_dist(const U& a, const V& b, int) const
      // {
      //     return a*b;
      // }
  };
}

#endif // DIST_UTILS_H