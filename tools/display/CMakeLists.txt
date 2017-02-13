#find_package(AntTweakBar REQUIRED)
#include(${ANT_TWEAK_BAR_INCLUDE_PATH})

find_package(OpenGL REQUIRED)
include_directories( ${OPENGL_INCLUDE_DIRS})

find_package(GLEW REQUIRED)
include_directories( ${GLEW_INCLUDE_PATH})

find_package(Freetype REQUIRED)
include_directories(${FREETYPE_INCLUDE_DIRS})

set(GLFW_BUILD_EXAMPLES OFF)
set(GLFW_BUILD_TESTS OFF)
set(GLFW_BUILD_DOCS OFF)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/extern/glfw/include)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/extern/glfw)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/extern)

SET(DISPLAY_TOOLS_SRC_FILES
	font_renderer.cc
	gl_manager.cpp
	gl_window.cpp
	glew_utils.cpp
)

SET(DISPLAY_TOOLS_HEADER_FILES
	font_renderer.h
	gl_manager.h
	gl_window.h
	glew_utils.h
)

add_library(tools_display ${DISPLAY_TOOLS_HEADER_FILES} ${DISPLAY_TOOLS_SRC_FILES})
target_link_libraries(tools_display ${FREETYPE_LIBRARIES} glfw ${GLFW_LIBRARIES} ${OPENGL_glu_LIBRARY} ${GLEW_LIBRARY} ${OPENGL_LIBRARIES})

add_library(tools_pcl_display pcl_window.h pcl_window.cpp)
target_link_libraries(tools_pcl_display tools_features tools_display ${PCL_IO_LIBRARIES} ${PCL_FEATURES_LIBRARIES})

add_custom_command(TARGET tools_display POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/display/shaders
        ${CMAKE_BINARY_DIR}/shaders)
        
add_custom_command(TARGET tools_display POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/display/fonts
        ${CMAKE_BINARY_DIR}/fonts)
