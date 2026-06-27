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

# library: rko_lio_ros_utils
add_subdirectory(utils)

# library: rko_lio_ros
add_library(rko_lio_ros STATIC
  base_node.cpp
  threaded_node.cpp
)

target_include_directories(rko_lio_ros PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/../..
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(rko_lio_ros
  PRIVATE nlohmann_json::nlohmann_json
  PUBLIC
    rko_lio_ros_utils
    rko_lio_core
    ${catkin_LIBRARIES}
    Sophus::Sophus
    Eigen3::Eigen
)

target_compile_features(rko_lio_ros PUBLIC cxx_std_20)

set_target_properties(rko_lio_ros PROPERTIES POSITION_INDEPENDENT_CODE ON)

# executable: online_node
add_executable(online_node online_node.cpp)
target_link_libraries(online_node PRIVATE rko_lio_ros)

# executable: online_imu_rate_node
add_executable(online_imu_rate_node online_imu_rate_node.cpp)
target_link_libraries(online_imu_rate_node PRIVATE rko_lio_ros)

# executable: offline_node
add_executable(offline_node offline_node.cpp)
target_link_libraries(offline_node PRIVATE rko_lio_ros)
