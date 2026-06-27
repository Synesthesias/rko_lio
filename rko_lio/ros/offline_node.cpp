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
#include "rko_lio/ros/utils/rosbag.hpp"
// other
#include <std_msgs/Float32MultiArray.h>

namespace rko_lio::ros {
class OfflineNode : public ThreadedNode {
public:
  std::unique_ptr<utils::BufferableBag> bag;

  ::ros::Publisher bag_progress_publisher;

  float total_bag_msgs = 0;
  float processed_bag_msgs = 0;
  std::chrono::steady_clock::time_point bag_start_time;

  OfflineNode()
      : ThreadedNode("rko_lio_offline_node"), bag_start_time(std::chrono::steady_clock::now()) {
    double skip_to_time = 0.0;
    pnh.param<double>("skip_to_time", skip_to_time, 0.0);

    std::string bag_path;
    if (!pnh.getParam("bag_path", bag_path)) {
      throw std::runtime_error("Parameter 'bag_path' is required for offline mode");
    }

    bag = std::make_unique<utils::BufferableBag>(
        bag_path,
        std::make_shared<utils::BufferableBag::TFBridge>(nh),
        std::vector<std::string>{imu_topic, lidar_topic},
        skip_to_time);
    total_bag_msgs = bag->message_count();
    bag_progress_publisher = nh.advertise<std_msgs::Float32MultiArray>("rko_lio/bag_progress", 10);
  }

  void publish_bag_progress() const {
    const auto now = std::chrono::steady_clock::now();
    const float elapsed_seconds = std::chrono::duration<float>(now - bag_start_time).count();

    const float percent_complete = 100.0F * processed_bag_msgs / total_bag_msgs;
    const float avg_time_per_msg = (processed_bag_msgs > 0) ? elapsed_seconds / processed_bag_msgs : 0.0F;
    const float seconds_remaining = avg_time_per_msg * (total_bag_msgs - processed_bag_msgs);

    std_msgs::Float32MultiArray progress_msg;
    progress_msg.layout.dim.resize(1);
    progress_msg.layout.dim[0].label = "percent_complete,seconds_remaining";
    progress_msg.layout.dim[0].size = 2;
    progress_msg.layout.dim[0].stride = 2;
    progress_msg.data = {percent_complete, seconds_remaining};

    bag_progress_publisher.publish(progress_msg);
  }

  void run() {
    while (::ros::ok() && !bag->finished()) {
      {
        if (lidar_buffer.size() >= 0.9 * max_lidar_buffer_size) {
          ROS_WARN_STREAM_ONCE("Lidar buffer size: " << lidar_buffer.size()
                                                     << ", max_lidar_buffer_size: " << max_lidar_buffer_size
                                                     << ", throttling the bag reading thread as it's too fast.\n");
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          continue;
        }
      }
      const utils::BagMessage bag_msg = bag->PopNextMessage();
      const auto& topic_name = bag_msg.topic_name;

      if (topic_name == imu_topic && bag_msg.imu_msg) {
        imu_callback(bag_msg.imu_msg);
      } else if (topic_name == lidar_topic && bag_msg.lidar_msg) {
        lidar_callback(bag_msg.lidar_msg);
      }

      processed_bag_msgs++;
      publish_bag_progress();
    }
    while (::ros::ok()) {
      {
        std::lock_guard<std::mutex> lock(buffer_mutex);
        if (lidar_buffer.empty() && !registration_busy.load()) {
          break;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
};
} // namespace rko_lio::ros

int main(int argc, char** argv) {
  const rko_lio::core::Timer timer("RKO LIO Offline Node");
  ::ros::init(argc, argv, "rko_lio_offline_node");
  rko_lio::ros::OfflineNode node;
  node.run();
  ::ros::shutdown();
  return 0;
}
