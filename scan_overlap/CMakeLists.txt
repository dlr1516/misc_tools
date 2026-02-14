cmake_minimum_required(VERSION 3.22)
project(scan_overlap)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_BUILD_TYPE Debug)

include_directories(src)

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

# ARS
find_package(ars REQUIRED)
message(STATUS "ars_FOUND ${ars_FOUND}")
message(STATUS "ars_INCLUDE_DIRS ${ars_INCLUDE_DIRS}")
message(STATUS "ars_LIBRARY_DIRS ${ars_LIBRARY_DIRS}")
message(STATUS "ars_LIBRARIES ${ars_LIBRARIES}")

if(${ars_FOUND})
  include_directories(${ars_INCLUDE_DIRS})
  link_directories(${ars_LIBRARY_DIRS})
endif()

# PCL
find_package(PCL 1.14 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

# VFC
add_library(vfc
  src/thirdparty/libvfc/vfc.cpp

  # src/thirdparty/libvfc/featureMatch.cpp # NOT USED anymore
)

# ICP
add_library(icp
  src/thirdparty/libicp/icp.cpp
  src/thirdparty/libicp/icpPointToPlane.cpp
  src/thirdparty/libicp/icpPointToPoint.cpp
  src/thirdparty/libicp/kdtree.cpp
  src/thirdparty/libicp/matrix.cpp
)

# # EXECUTABLES
add_executable(find_overlap_graph
  src/find_overlap_graph.cpp
  src/scan_overlap.cpp
  src/transform_utils.cpp
)
target_link_libraries(find_overlap_graph ${Boost_LIBRARIES} ${rofl_LIBRARIES} opencv_core)
target_compile_features(find_overlap_graph PRIVATE cxx_std_17)

add_executable(run_graph_tests
  app/run_graph_tests.cpp
  src/scan_overlap.cpp
  src/transform_utils.cpp
  src/simple_icp_registration.cpp
  src/vfc_registration.cpp
  src/thirdparty/libvfc/vfc.cpp
  src/thirdparty/libicp/icp.cpp
)
target_link_libraries(run_graph_tests ${Boost_LIBRARIES} ${rofl_LIBRARIES} ${ars_LIBRARIES} icp vfc opencv_core ${PCL_LIBRARIES})
target_compile_features(run_graph_tests PRIVATE cxx_std_17)

add_executable(test_run_graph_tests
  app/run_graph_tests.cpp
  src/scan_overlap.cpp
  src/transform_utils.cpp
  src/simple_icp_registration.cpp
  src/vfc_registration.cpp
  src/thirdparty/libvfc/vfc.cpp
  src/thirdparty/libicp/icp.cpp
)
target_link_libraries(test_run_graph_tests ${Boost_LIBRARIES} ${rofl_LIBRARIES} ${ars_LIBRARIES} icp vfc opencv_core ${PCL_LIBRARIES})
target_compile_features(test_run_graph_tests PRIVATE cxx_std_17)

add_executable(draw_graph_corr
  app/draw_graph_corr.cpp
  src/transform_utils.cpp
)
target_link_libraries(draw_graph_corr ${Boost_LIBRARIES} ${rofl_LIBRARIES} ${ars_LIBRARIES} ${PCL_LIBRARIES})
target_compile_features(draw_graph_corr PRIVATE cxx_std_17)
