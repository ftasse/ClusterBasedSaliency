SET(MISC_TOOLS_SRC_FILES
  cmd_parser.cc
  cgal_utils.cpp
  pcl_utils.cc
  eigen3_utils.cpp
  shading_utils.cc
)

SET(MISC_TOOLS_HEADER_FILES
  cmd_parser.h
  cgal_utils.h
  pcl_utils.h
  flann_utils.h
  eigen3_utils.h
  shading_utils.h
  matrix_io.h
)

add_library(tools_misc ${MISC_TOOLS_HEADER_FILES} ${MISC_TOOLS_SRC_FILES})
target_link_libraries(tools_misc ${PCL_FEATURES_LIBRARIES} ${CGAL_LIBRARIES})
