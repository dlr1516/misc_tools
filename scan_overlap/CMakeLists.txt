cmake_minimum_required(VERSION 3.0.2)
project(scan_overlap)

set(CMAKE_BUILD_TYPE Debug)

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Boost
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
message("Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

# ROFL
find_package(rofl REQUIRED)
include_directories(${rofl_INCLUDE_DIRS})

include_directories(src)

add_library(vfc
  src/thirdparty/libvfc/vfc.cpp

  # src/thirdparty/libvfc/featureMatch.cpp # NOT USED anymore
)

add_library(icp
  src/thirdparty/libicp/icp.cpp
  src/thirdparty/libicp/icpPointToPlane.cpp
  src/thirdparty/libicp/icpPointToPoint.cpp
  src/thirdparty/libicp/kdtree.cpp
  src/thirdparty/libicp/matrix.cpp
)

add_executable(find_overlap_graph
  src/find_overlap_graph.cpp
  src/scan_overlap.cpp
  src/transform_utils.cpp
)
target_link_libraries(find_overlap_graph ${Boost_LIBRARIES} ${rofl_LIBRARIES} opencv_core)
target_compile_features(find_overlap_graph PRIVATE cxx_std_17 )

add_executable(run_graph_tests
  app/run_graph_tests.cpp
  src/scan_overlap.cpp
  src/transform_utils.cpp
  src/thirdparty/libvfc/vfc.cpp
  src/thirdparty/libicp/icp.cpp
)
target_link_libraries(run_graph_tests ${Boost_LIBRARIES} ${rofl_LIBRARIES} icp vfc opencv_core)
target_compile_features(run_graph_tests PRIVATE cxx_std_17)