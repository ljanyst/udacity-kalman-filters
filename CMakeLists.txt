project(Kalman)
cmake_minimum_required (VERSION 3.5)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_definitions(-std=c++0x)

add_library(Kalman
  STATIC
  src/kalman_filter.cc
  src/utils.cc)

add_executable(ExtendedKF
  src/main_ekf.cc
  src/lidar_radar_ekf.cc)

target_link_libraries(ExtendedKF Kalman)

add_executable(UnscentedKF
  src/main_ukf.cc
  src/lidar_radar_ukf.cc)

target_link_libraries(UnscentedKF Kalman)
