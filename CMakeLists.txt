# MIT License
#
# Copyright (c) 2025 Meher V.R. Malladi.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

cmake_minimum_required(VERSION 3.16)
project(rko_lio LANGUAGES CXX C)

if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()

option(RKO_LIO_BUILD_ROS "Build the ROS wrapper for RKO LIO" ON)
option(RKO_LIO_BUILD_TESTS "Build core C++ tests" OFF)

include(cmake/compat_target_sources.cmake)

# Find catkin and ROS dependencies first (before any FetchContent)
if(RKO_LIO_BUILD_ROS)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    tf2
    tf2_ros
    sensor_msgs
    nav_msgs
    geometry_msgs
    rosbag
    tf2_msgs
  )

  catkin_package(
    INCLUDE_DIRS rko_lio
    LIBRARIES rko_lio.core rko_lio.ros rko_lio.ros.utils
    CATKIN_DEPENDS roscpp std_msgs tf2 tf2_ros sensor_msgs nav_msgs geometry_msgs rosbag tf2_msgs
    DEPENDS Eigen3
  )

  include_directories(${catkin_INCLUDE_DIRS})
endif()

# Non-ROS dependencies: find system packages first, FetchContent as fallback
find_package(Eigen3 3.4 REQUIRED CONFIG)
find_package(TBB REQUIRED CONFIG)

# Sophus: try system/ros paths, then FetchContent
find_package(Sophus QUIET CONFIG)
if(NOT Sophus_FOUND)
  message(STATUS "Sophus not found via find_package, using FetchContent")
  include(FetchContent)
  set(SOPHUS_USE_BASIC_LOGGING ON CACHE BOOL "")
  set(BUILD_SOPHUS_TESTS OFF CACHE BOOL "")
  set(BUILD_SOPHUS_EXAMPLES OFF CACHE BOOL "")
  set(BUILD_PYTHON_BINDINGS OFF CACHE BOOL "")
  if(CMAKE_VERSION VERSION_LESS "3.24")
    set(SOPHUS_VERSION "1.22.10")
  else()
    set(SOPHUS_VERSION "1.24.6")
  endif()
  FetchContent_Declare(
    Sophus
    URL https://github.com/strasdat/Sophus/archive/refs/tags/${SOPHUS_VERSION}.tar.gz)
  FetchContent_MakeAvailable(Sophus)
endif()

# tsl-robin-map: try system, then FetchContent
find_package(tsl-robin-map QUIET CONFIG)
if(NOT tsl-robin-map_FOUND)
  message(STATUS "tsl-robin-map not found via find_package, using FetchContent")
  if(NOT COMMAND FetchContent_Declare)
    include(FetchContent)
  endif()
  FetchContent_Declare(
    tsl-robin-map
    URL https://github.com/Tessil/robin-map/archive/refs/tags/v1.4.0.tar.gz)
  FetchContent_MakeAvailable(tsl-robin-map)
endif()

add_subdirectory(rko_lio/core)

if(RKO_LIO_BUILD_ROS)
  message(STATUS "Building RKO_LIO ROS wrapper.")

  find_package(nlohmann_json REQUIRED CONFIG)

  add_subdirectory(rko_lio/ros)

  install(
    TARGETS online_node online_imu_rate_node offline_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endif()

if(RKO_LIO_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests/core)
endif()
