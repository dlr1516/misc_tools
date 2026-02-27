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

# g2o
find_package(g2o REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})
link_directories(${G2O_LIBRARY_DIRS})

# Ceres
find_package(Ceres REQUIRED)

# Absl
# find_package(absl REQUIRED)
# include_directories(${absl_INCLUDE_DIRS})
# link_directories(${absl_LIBRARY_DIRS})

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
# find_overlap_graph
add_executable(find_overlap_graph
  src/find_overlap_graph.cpp
  src/scan_overlap.cpp
  src/transform_utils.cpp
)
target_link_libraries(find_overlap_graph ${Boost_LIBRARIES} ${rofl_LIBRARIES} opencv_core)
target_compile_features(find_overlap_graph PRIVATE cxx_std_17)

# run_graph_tests
add_executable(run_graph_tests
  app/run_graph_tests.cpp
  src/scan_overlap.cpp
  src/transform_utils.cpp
  src/simple_icp_registration.cpp
  src/vfc_registration.cpp
  src/thirdparty/libvfc/vfc.cpp
  src/thirdparty/libicp/icp.cpp
)
target_link_libraries(run_graph_tests ${Boost_LIBRARIES} ${rofl_LIBRARIES} ${ars_LIBRARIES} icp vfc opencv_core ${PCL_LIBRARIES}
  g2o::core g2o::solver_eigen g2o::types_slam2d g2o::types_slam3d g2o::types_sba g2o::solver_csparse)
target_compile_features(run_graph_tests PRIVATE cxx_std_17)

# test_run_graph_tests
add_executable(test_run_graph_tests
  app/test_run_graph_tests.cpp
  src/scan_overlap.cpp
  src/transform_utils.cpp
  src/simple_icp_registration.cpp
  src/vfc_registration.cpp
  src/thirdparty/libvfc/vfc.cpp
  src/thirdparty/libicp/icp.cpp
)
target_link_libraries(test_run_graph_tests ${Boost_LIBRARIES} ${rofl_LIBRARIES} ${ars_LIBRARIES} icp vfc opencv_core ${PCL_LIBRARIES})
target_compile_features(test_run_graph_tests PRIVATE cxx_std_17)

# test_ars_pw
add_executable(test_ars_pw
  app/test_ars_pw.cpp
  src/scan_overlap.cpp
  src/transform_utils.cpp
  src/simple_icp_registration.cpp
  src/vfc_registration.cpp
  src/thirdparty/libvfc/vfc.cpp
  src/thirdparty/libicp/icp.cpp
)
target_link_libraries(test_ars_pw ${Boost_LIBRARIES} ${rofl_LIBRARIES} ${ars_LIBRARIES} icp vfc opencv_core ${PCL_LIBRARIES})
target_compile_features(test_ars_pw PRIVATE cxx_std_17)

# draw_graph_corr
add_executable(draw_graph_corr
  app/draw_graph_corr.cpp
  src/transform_utils.cpp
)
target_link_libraries(draw_graph_corr ${Boost_LIBRARIES} ${rofl_LIBRARIES} ${ars_LIBRARIES} ${PCL_LIBRARIES})
target_compile_features(draw_graph_corr PRIVATE cxx_std_17)

# test_pgo_g2o_tutorial
add_library(tutorial_slam2d_library ${G2O_LIB_TYPE}
  src/g2o/edge_se2_pointxy.cpp src/g2o/se2.h src/g2o/vertex_point_xy.h
  src/g2o/edge_se2.cpp src/g2o/edge_se2_pointxy.h src/g2o/vertex_se2.cpp
  src/g2o/edge_se2.h src/g2o/vertex_point_xy.cpp src/g2o/vertex_se2.h
  src/g2o/parameter_se2_offset.h src/g2o/parameter_se2_offset.cpp
  src/g2o/types_tutorial_slam2d.h src/g2o/types_tutorial_slam2d.cpp
  src/g2o/simulator.h src/g2o/simulator.cpp
  src/g2o/g2o_tutorial_slam2d_api.h
)

set_target_properties(tutorial_slam2d_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}tutorial_slam2d)
set_target_properties(tutorial_slam2d_library PROPERTIES
  VERSION ${G2O_LIB_VERSION}
  SOVERSION ${G2O_LIB_SOVERSION})

target_link_libraries(tutorial_slam2d_library g2o::core g2o::solver_eigen)

add_executable(test_pgo_g2o_tutorial
  app/test_pgo_g2o_tutorial.cpp
)

set_target_properties(test_pgo_g2o_tutorial PROPERTIES OUTPUT_NAME tutorial_slam2d)

target_link_libraries(test_pgo_g2o_tutorial tutorial_slam2d_library)

# test_pgo_g2o
add_executable(test_pgo_g2o
  app/test_pgo_g2o.cpp
)
target_link_libraries(test_pgo_g2o ${Boost_LIBRARIES} ${rofl_LIBRARIES} ${ars_LIBRARIES} ${PCL_LIBRARIES} g2o::core g2o::solver_eigen g2o::types_slam2d g2o::types_slam3d g2o::types_sba g2o::solver_csparse)
target_compile_features(test_pgo_g2o PRIVATE cxx_std_17)

# test_pgo_g2o_arsgraph
add_executable(test_pgo_g2o_arsgraph
  app/test_pgo_g2o_arsgraph.cpp
  src/transform_utils.cpp
  src/simple_icp_registration.cpp
)
target_link_libraries(test_pgo_g2o_arsgraph ${Boost_LIBRARIES} ${rofl_LIBRARIES} ${ars_LIBRARIES} icp ${PCL_LIBRARIES}
  g2o::core g2o::solver_eigen g2o::types_slam2d g2o::types_slam3d g2o::types_sba g2o::solver_csparse)
target_compile_features(test_pgo_g2o_arsgraph PRIVATE cxx_std_17)

# test_pgo_ceres
add_executable(test_pgo_ceres
  app/test_pgo_ceres.cpp
)
target_link_libraries(test_pgo_ceres ${Boost_LIBRARIES} ${rofl_LIBRARIES} ${ars_LIBRARIES} ${PCL_LIBRARIES} Ceres::ceres)
target_compile_features(test_pgo_ceres PRIVATE cxx_std_17)

# test_hough_spectrum
add_executable(test_hough_spectrum
  app/test_hough_spectrum.cpp
  src/scan_overlap.cpp
  src/transform_utils.cpp
  src/simple_icp_registration.cpp
  src/thirdparty/libicp/icp.cpp
)
target_link_libraries(test_hough_spectrum ${Boost_LIBRARIES} ${rofl_LIBRARIES} ${ars_LIBRARIES} icp ${PCL_LIBRARIES})
target_compile_features(test_hough_spectrum PRIVATE cxx_std_17)

# test_ars_graph_time
add_executable(test_ars_graph_time
  app/test_ars_graph_time.cpp
  src/transform_utils.cpp
)
target_link_libraries(test_ars_graph_time ${Boost_LIBRARIES} ${rofl_LIBRARIES} ${ars_LIBRARIES})
target_compile_features(test_ars_graph_time PRIVATE cxx_std_17)