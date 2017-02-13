cmake_minimum_required(VERSION 2.8.11)

project(shape3d_saliency)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(tools)

add_subdirectory(tools)

find_package(PCL 1.7 REQUIRED COMPONENTS common io features surface segmentation) # visualization  keypoints
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

# # LIST(FILTER PCL_IO_LIBRARIES ".+\\Qt.+" OUTPUT_VARIABLE PCL_IO_LIBRARIES)
# FOREACH(item ${PCL_IO_LIBRARIES})
#   # STRING(REGEX MATCH ".+\\Qt.+" item ${item})
#   IF( NOT(item MATCHES ".+\\Qt.*"))
#     # message(status " ex: ${item}")
#     LIST(APPEND CLEAN_PCL_IO_LIBRARIES ${item})
#   ENDIF(NOT(item MATCHES ".+\\Qt.*"))
# ENDFOREACH(item ${PCL_IO_LIBRARIES})
# set(PCL_IO_LIBRARIES ${CLEAN_PCL_IO_LIBRARIES})

add_subdirectory(saliency)
add_subdirectory(segmentation)

add_executable(generate_pcl_saliency  generate_pcl_saliency.cc)
target_link_libraries(generate_pcl_saliency saliency)

add_executable(segmentation_main segmentation_main.cc)
target_link_libraries(segmentation_main segmentation)

# Make bundles

add_executable(generate_saliency MACOSX_BUNDLE generate_pcl_saliency.cc)
target_link_libraries(generate_saliency saliency)

# SET(APPS "${CMAKE_BINARY_DIR}/generate_saliency.app")
# set(YOUR_PLUGINS "") #empty for now
# set(YOUR_LIBRARY_DIRS "")

# set(BU_CHMOD_BUNDLE_ITEMS ON)
# include(BundleUtilities) 
# fixup_bundle(${APPS} "" "") # \"${YOUR_PLUGINS}\" \"${YOUR_LIBRARY_DIRS}\")
# VERIFY_APP(${APPS})

SET(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR})
set(CMAKE_SKIP_INSTALL_ALL_DEPENDENCY true)

SET(APPS "${CMAKE_INSTALL_PREFIX}/bin/generate_saliency.app")
install (TARGETS generate_saliency DESTINATION bin)
INSTALL(CODE "
   include(BundleUtilities)
   fixup_bundle(\"${APPS}\"   \"\"   \"${DIRS}\")
   " COMPONENT Runtime)