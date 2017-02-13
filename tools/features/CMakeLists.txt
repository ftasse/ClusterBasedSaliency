SET(FEATURES_SRC_FILES
  normals.cc descriptors.cc gabor.cc)

SET(FEATURES_HEADER_FILES
  normals.h descriptors.h gabor.h)

IF(BUILD_HKS)
add_subdirectory(sihks)
ENDIF(BUILD_HKS)

add_library(tools_features ${FEATURES_SRC_FILES} ${FEATURES_HEADER_FILES})
target_link_libraries(tools_features ${PCL_FEATURES_LIBRARIES} ${CGAL_LIBRARIES})
