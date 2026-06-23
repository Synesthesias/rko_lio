// MIT License

// Copyright (c) 2024 Tiziano Guadagnino, Benedikt Mersch, Ignacio Vizzo, Cyrill
// Stachniss.

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once
// tf2
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
// ROS
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>
// stl
#include <chrono>
#include <cstddef>
#include <memory>
#include <queue>
#include <string>
#include <variant>
#include <vector>

namespace rko_lio::ros::utils {

struct BagMessage {
  std::string topic_name;
  ::ros::Time stamp;
  std::variant<sensor_msgs::Imu::ConstPtr, sensor_msgs::PointCloud2::ConstPtr> data;
};

class BufferableBag {
public:
  struct TFBridge {
    explicit TFBridge(::ros::NodeHandle& nh);
    void ProcessTFMessage(const tf2_msgs::TFMessage& tf_message, const std::string& topic_name) const;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster;
  };

  BufferableBag(const std::string& bag_path,
                const std::shared_ptr<TFBridge> tf_bridge,
                const std::vector<std::string>& topics,
                const ::ros::Duration seek = ::ros::Duration(0.0),
                const std::chrono::seconds buffer_size = std::chrono::seconds(1));

  void publish_tf_static(const std::string& bag_path);
  size_t message_count() const;
  void BufferMessages();
  BagMessage PopNextMessage();
  bool finished() const;
  void close();

private:
  std::shared_ptr<TFBridge> tf_bridge_;
  std::unique_ptr<rosbag::Bag> bag_;
  std::unique_ptr<rosbag::View> view_;
  rosbag::View::iterator view_it_;
  bool view_exhausted_ = false;
  std::queue<BagMessage> buffer_;
  std::chrono::seconds buffer_size_;
  std::vector<std::string> topics_;
  size_t message_count_{0};
};
} // namespace rko_lio::ros::utils
