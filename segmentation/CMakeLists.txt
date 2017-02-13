SET(SEGMENTATION_SRC_FILES
  segmentation_utils.cc
  enhanced_supervoxel_clustering.cc
)

SET(SEGMENTATION_HEADER_FILES
  segment.h
  segmentation_utils.h
  enhanced_supervoxel_clustering.h
)

add_library(segmentation ${SEGMENTATION_HEADER_FILES} ${SEGMENTATION_SRC_FILES})
target_link_libraries(segmentation tools_features tools_misc tools_io ${PCL_SEGMENTATION_LIBRARIES})
