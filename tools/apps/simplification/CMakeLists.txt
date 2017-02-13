SET(SIMPLIFICATION_SRC_FILES salient_based_simplification.cpp)
SET(SIMPLIFICATION_HEADER_FILES salient_based_simplification.h)

add_library(simplification ${SIMPLIFICATION_HEADER_FILES} ${SIMPLIFICATION_SRC_FILES})
target_link_libraries(simplification tools_features tools_misc tools_io 
  ${PCL_LIBRARIES} ${CGAL_LIBRARIES}
)