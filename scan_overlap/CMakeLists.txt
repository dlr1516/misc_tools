cmake_minimum_required(VERSION 3.0.2)
project(scan_overlap)

find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(Eigen3 REQUIRED)
find_package(rofl REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${rofl_INCLUDE_DIRS})
include_directories(src)

add_executable(scan_overlap
    src/scan_overlap.cpp
    src/transform_utils.cpp
)
target_link_libraries(scan_overlap ${Boost_LIBRARIES} ${rofl_LIBRARIES})
target_compile_features(scan_overlap PRIVATE cxx_std_11)
