#ifndef SALIENCY_OUTPUT_DETAILS_H
#define SALIENCY_OUTPUT_DETAILS_H

#include <string>
#include "saliency/clustering_saliency_detection.h"

namespace pcl {
  class PolygonMesh;
}

void saveSaliencyIntermediateResults(
  const pcl::PolygonMesh &mesh,
  const ClusterBasedSaliencyDetection::IntermediateData &intermediate_data,
  std::string prefix = "");

#endif // SALIENCY_OUTPUT_DETAILS_H