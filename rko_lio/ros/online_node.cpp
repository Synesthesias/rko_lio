/*
 * MIT License
 *
 * Copyright (c) 2025 Meher V.R. Malladi.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "threaded_node.hpp"
#include "rko_lio/core/profiler.hpp"

namespace rko_lio::ros {
class OnlineNode : public ThreadedNode {
public:
  ::ros::Subscriber imu_sub;
  ::ros::Subscriber lidar_sub;
  rko_lio::core::Timer timer;

  OnlineNode(const OnlineNode&) = delete;
  OnlineNode(OnlineNode&&) = delete;
  OnlineNode& operator=(const OnlineNode&) = delete;
  OnlineNode& operator=(OnlineNode&&) = delete;

  OnlineNode() : ThreadedNode("rko_lio_online_node"), timer("RKO LIO Online Node") {
    imu_sub = nh.subscribe<sensor_msgs::Imu>(
        imu_topic, 100, [this](const sensor_msgs::Imu::ConstPtr& msg) { imu_callback(msg); });

    lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>(
        lidar_topic, 10, [this](const sensor_msgs::PointCloud2::ConstPtr& msg) { lidar_callback(msg); });
  }
};
} // namespace rko_lio::ros

int main(int argc, char** argv) {
  ::ros::init(argc, argv, "rko_lio_online_node");
  rko_lio::ros::OnlineNode node;
  ::ros::AsyncSpinner spinner(2);
  spinner.start();
  ::ros::waitForShutdown();
  return 0;
}
