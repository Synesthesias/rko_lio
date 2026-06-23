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

#include "rosbag.hpp"
// stl
#include <algorithm>
#include <iostream>

namespace rko_lio::ros::utils {
// TFBridge----------------------------------------------------------------------------------------
BufferableBag::TFBridge::TFBridge(::ros::NodeHandle& nh) {
  tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>();
  tf_static_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>();
}

void BufferableBag::TFBridge::ProcessTFMessage(const tf2_msgs::TFMessage& tf_message,
                                               const std::string& topic_name) const {
  for (const auto& transform : tf_message.transforms) {
    if (topic_name == "/tf_static") {
      tf_static_broadcaster->sendTransform(transform);
    } else {
      tf_broadcaster->sendTransform(transform);
    }
  }
}

// BufferableBag-----------------------------------------------------------------------------------
BufferableBag::BufferableBag(const std::string& bag_path,
                             const std::shared_ptr<TFBridge> tf_bridge,
                             const std::vector<std::string>& topics,
                             const ::ros::Duration seek,
                             const std::chrono::seconds buffer_size)
    : tf_bridge_(tf_bridge),
      buffer_size_(buffer_size),
      topics_(topics) {
  publish_tf_static(bag_path);

  bag_ = std::make_unique<rosbag::Bag>();
  bag_->open(bag_path, rosbag::bagmode::Read);

  std::vector<std::string> all_topics = topics_;
  if (std::find(all_topics.begin(), all_topics.end(), "/tf") == all_topics.end()) {
    all_topics.push_back("/tf");
  }

  view_ = std::make_unique<rosbag::View>(*bag_, rosbag::TopicQuery(all_topics));

  // Count messages for the requested topics only
  {
    rosbag::View count_view(*bag_, rosbag::TopicQuery(topics_));
    message_count_ = count_view.size();
  }

  view_it_ = view_->begin();

  // Skip messages until we reach the seek time
  if (seek.toSec() > 0.0) {
    const ::ros::Time seek_time = view_->getBeginTime() + seek;
    while (view_it_ != view_->end() && (*view_it_).getTime() < seek_time) {
      const rosbag::MessageInstance& m = *view_it_;
      if (m.getTopic() == "/tf") {
        auto tf_msg = m.instantiate<tf2_msgs::TFMessage>();
        if (tf_msg) {
          tf_bridge_->ProcessTFMessage(*tf_msg, m.getTopic());
        }
      }
      ++view_it_;
    }
  }

  std::cout << "Bag reader initialized with total message count: " << message_count_ << '\n';
  BufferMessages();
}

void BufferableBag::publish_tf_static(const std::string& bag_path) {
  std::cout << "Opening the bag first to publish all the tf_static messages\n";
  rosbag::Bag tf_bag;
  tf_bag.open(bag_path, rosbag::bagmode::Read);
  rosbag::View tf_view(tf_bag, rosbag::TopicQuery(std::vector<std::string>{"/tf_static"}));
  for (const rosbag::MessageInstance& m : tf_view) {
    auto tf_msg = m.instantiate<tf2_msgs::TFMessage>();
    if (tf_msg) {
      tf_bridge_->ProcessTFMessage(*tf_msg, "/tf_static");
    }
  }
  tf_bag.close();
  std::cout << "tf_static published, if any. Closing the bag...\n";
}

bool BufferableBag::finished() const { return view_exhausted_ && buffer_.empty(); }
void BufferableBag::close() {
  if (bag_) {
    bag_->close();
  }
}

size_t BufferableBag::message_count() const { return message_count_; }

void BufferableBag::BufferMessages() {
  auto buffer_is_filled = [&]() -> bool {
    if (buffer_.empty()) {
      return false;
    }
    const auto first_ns = buffer_.front().stamp.toNSec();
    const auto last_ns = buffer_.back().stamp.toNSec();
    return (last_ns - first_ns) > static_cast<uint64_t>(
               std::chrono::duration_cast<std::chrono::nanoseconds>(buffer_size_).count());
  };

  while (!buffer_is_filled() && view_it_ != view_->end()) {
    const rosbag::MessageInstance& m = *view_it_;
    const std::string& topic = m.getTopic();

    if (topic == "/tf") {
      auto tf_msg = m.instantiate<tf2_msgs::TFMessage>();
      if (tf_msg) {
        tf_bridge_->ProcessTFMessage(*tf_msg, topic);
      }
    } else if (std::find(topics_.cbegin(), topics_.cend(), topic) != topics_.end()) {
      BagMessage bag_msg;
      bag_msg.topic_name = topic;
      bag_msg.stamp = m.getTime();

      auto imu_msg = m.instantiate<sensor_msgs::Imu>();
      if (imu_msg) {
        bag_msg.data = imu_msg;
        buffer_.push(std::move(bag_msg));
      } else {
        auto pc_msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (pc_msg) {
          bag_msg.data = pc_msg;
          buffer_.push(std::move(bag_msg));
        }
      }
    }
    ++view_it_;
  }

  if (view_it_ == view_->end()) {
    view_exhausted_ = true;
  }
}

BagMessage BufferableBag::PopNextMessage() {
  BagMessage msg = std::move(buffer_.front());
  buffer_.pop();
  if (!view_exhausted_) {
    BufferMessages();
  }
  return msg;
}
} // namespace rko_lio::ros::utils
