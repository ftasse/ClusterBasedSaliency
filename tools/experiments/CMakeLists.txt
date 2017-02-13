add_executable(convert_shapeset_clusters convert_shapeset_clusters.cc)
target_link_libraries(convert_shapeset_clusters 
  tools_io trimesh 
  ${PCL_COMMON_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES} ${PCL_IO_LIBRARIES})