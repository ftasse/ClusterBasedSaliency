include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../extern)

SET(MINCUT_PARTITIONING_SRC_FILES
	${CMAKE_CURRENT_SOURCE_DIR}/../extern/gco/GCoptimization.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../extern/gco/LinkedBlockList.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../extern/gco/graph.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../extern/gco/maxflow.cpp
  segmentation.cpp
)

SET(MINCUT_PARTITIONING_HEADER_FILES
	segmentation.h
	${CMAKE_CURRENT_SOURCE_DIR}/../extern/gco/GCoptimization.h
  ${CMAKE_CURRENT_SOURCE_DIR}/../extern/gco/LinkedBlockList.h
  ${CMAKE_CURRENT_SOURCE_DIR}/../extern/gco/graph.h
  ${CMAKE_CURRENT_SOURCE_DIR}/../extern/gco/energy.h
  ${CMAKE_CURRENT_SOURCE_DIR}/../extern/gco/block.h
)

add_library(mincut_partitioning
            ${MINCUT_PARTITIONING_HEADER_FILES} 
            ${MINCUT_PARTITIONING_SRC_FILES})
target_link_libraries(mincut_partitioning vlfeat
  tools_features tools_misc tools_io  
  ${PCL_LIBRARIES} ${CGAL_LIBRARIES}
)