SET(IO_SRC_FILES
  off_io.cc
)

SET(IO_HEADER_FILES
  off_io.h 
)

add_library(tools_io ${IO_HEADER_FILES} ${IO_SRC_FILES})
target_link_libraries(tools_io  ${PCL_IO_LIBRARIES})

add_library(tools_io_psb psb_classification.h psb_classification.cpp)