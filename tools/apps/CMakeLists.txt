include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(mincut_partitioning)
add_subdirectory(simplification)
add_subdirectory(extern/vlfeat)

add_executable(mesh_viewer mesh_viewer.cc)
target_link_libraries(mesh_viewer tools_features tools_pcl_display  tools_misc tools_io )

add_executable(simplification_main simplification_main.cc)
target_link_libraries(simplification_main simplification)

add_executable(partitioning_main partitioning_main.cc)
target_link_libraries(partitioning_main mincut_partitioning)

add_executable(generate_pcl_filelist generate_pcl_filelist.cc)
target_link_libraries(generate_pcl_filelist tools_io_psb tools_misc ${Boost_LIBRARIES})

add_executable(generate_pcl_sampling generate_pcl_sampling.cc)
target_link_libraries(generate_pcl_sampling
  tools_io tools_features tools_misc ${PCL_LIBRARIES}
)

add_executable(generate_random_points generate_random_points.cc)
target_link_libraries(generate_random_points
  tools_io tools_misc ${PCL_LIBRARIES}
)

add_executable(generate_pcl_normals generate_pcl_normals.cc)
target_link_libraries(generate_pcl_normals
  tools_io tools_features tools_misc ${PCL_LIBRARIES}
)

add_executable(convert_format convert_format.cc)
target_link_libraries(convert_format
  tools_io tools_misc ${PCL_LIBRARIES}
)

include_directories(extern)
add_executable(generate_pcl_views generate_pcl_views.cc 
  extern/pcl_apps/render_views_tesselated_sphere.cpp
  )
target_link_libraries(generate_pcl_views
  tools_io tools_misc ${PCL_LIBRARIES}
)

add_executable(fixed_virtual_scanner fixed_virtual_scanner.cc)
target_link_libraries(fixed_virtual_scanner ${PCL_LIBRARIES})

add_executable(generate_pcl_descriptors generate_pcl_descriptors.cc)
target_link_libraries(generate_pcl_descriptors
  tools_io tools_features tools_misc ${PCL_LIBRARIES}
)

add_executable(generate_d2_descriptors generate_d2_descriptors)
target_link_libraries(generate_d2_descriptors
  tools_io tools_misc ${PCL_LIBRARIES}
)