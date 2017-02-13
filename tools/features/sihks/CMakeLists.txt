
set (CMAKE_MODULE_PATH  "${CMAKE_CURRENT_SOURCE_DIR}/" ${CMAKE_MODULE_PATH})

find_package(OpenMesh)
find_package(ARPACK)
# find_package(Cholmod)

include_directories(${OPENMESH_INCLUDE_DIR} ${CHOLMOD_INCLUDES})

SET(SIHKS_SRC_FILES
  eigendecomposition.cpp hks.cc)

SET(SIHKS_HEADER_FILES
  eigendecomposition.h generate_lpmatrix.h hks.h)

add_library(tools_features_sihks ${SIHKS_SRC_FILES} ${SIHKS_HEADER_FILES})
target_link_libraries(tools_features_sihks ${PCL_FEATURES_LIBRARIES} ${CGAL_LIBRARIES} ${OPENMESH_LIBRARIES} ${ARPACK_LIBRARY} ${CHOLMOD_LIBRARIES})
