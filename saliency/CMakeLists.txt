find_package(PCL 1.7 REQUIRED COMPONENTS common io features surface segmentation)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

SET(SALIENCY_SRC_FILES 
    clustering_saliency_detection.cpp
    saliency_output_details.cc)
SET(SALIENCY_HEADER_FILES 
    clustering_saliency_detection.h
    saliency_output_details.h)

add_library(saliency ${SALIENCY_HEADER_FILES} ${SALIENCY_SRC_FILES})
target_link_libraries(saliency tools_io tools_features tools_misc  segmentation ) #${PCL_LIBRARIES} ${FLANN_LIBRARIES} ${CGAL_LIBRARIES}