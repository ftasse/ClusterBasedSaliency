cmake_minimum_required(VERSION 2.8)

project(vlfeat)

# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include_directories(libvlfeat/vl)

add_definitions(-msse2)

if (ENABLE_AVX)
  # check_c_source_runs("
  #   #include <immintrin.h>
  #   int main()
  #   {
  #   __m256i a = _mm256_setzero_si256();
  #   return 0;
  #   }"
  #   CC_HAS_AVX_INTRINSICS)

  if (!CC_HAS_AVX_INTRINSICS)
      message(WARNING "AVX is not supported")
      add_definitions(-DVL_DISABLE_AVX)
  else()
      add_definitions(-mavx)
      # add_compile_flags("C;CXX" "-mavx")
      message(STATUS "AVX is enabled - target CPU must support it")
  endif()
else()
  add_definitions(-DVL_DISABLE_AVX)
endif()

set(vl_sources
    
	libvlfeat/vl/aib.c 
  libvlfeat/vl/array.c 
  libvlfeat/vl/covdet.c 
  libvlfeat/vl/dsift.c 
  libvlfeat/vl/fisher.c 
  libvlfeat/vl/generic.c 
  libvlfeat/vl/getopt_long.c 
  libvlfeat/vl/gmm.c 
  libvlfeat/vl/hikmeans.c 
  libvlfeat/vl/hog.c 
  libvlfeat/vl/homkermap.c 
  libvlfeat/vl/host.c 
  libvlfeat/vl/ikmeans.c 
  libvlfeat/vl/imopv.c 
  libvlfeat/vl/imopv_sse2.c 
  libvlfeat/vl/kdtree.c 
  libvlfeat/vl/kmeans.c 
  libvlfeat/vl/lbp.c 
  libvlfeat/vl/liop.c 
  libvlfeat/vl/mathop.c 
  libvlfeat/vl/mathop_sse2.c
  libvlfeat/vl/mathop_avx.c
  libvlfeat/vl/mser.c 
  libvlfeat/vl/pgm.c 
  libvlfeat/vl/quickshift.c 
  libvlfeat/vl/random.c 
  libvlfeat/vl/rodrigues.c 
  libvlfeat/vl/scalespace.c 
  libvlfeat/vl/sift.c 
  libvlfeat/vl/slic.c 
  libvlfeat/vl/stringop.c 
  libvlfeat/vl/svm.c 
  libvlfeat/vl/svmdataset.c 
  libvlfeat/vl/vlad.c)

set_source_files_properties(${vl_sources} PROPERTIES LANGUAGE C)

add_library(vlfeat ${vl_sources})

