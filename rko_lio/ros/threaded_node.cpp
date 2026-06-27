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
#include "rko_lio/core/process_timestamps.hpp"
#include "rko_lio/core/profiler.hpp"
#include "rko_lio/ros/utils/utils.hpp"
// other
#include <stdexcept>

namespace {
using namespace std::literals;
} // namespace

namespace rko_lio::ros {

ThreadedNode::ThreadedNode(const std::string& node_name) : BaseNode(node_name) {
  int max_buf = static_cast<int>(max_lidar_buffer_size);
  pnh.param<int>("async/max_lidar_buffer_size", max_buf, max_buf);
  max_lidar_buffer_size = static_cast<size_t>(max_buf);
  registration_thread = std::thread([this]() { registration_loop(); });
}

void ThreadedNode::imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
  if (!ensure_frame_and_extrinsics(imu_frame, imu_msg->header.frame_id, "IMU")) {
    return;
  }
  {
    std::lock_guard lock(buffer_mutex);
    imu_buffer.emplace(imu_msg_to_imu_data(*imu_msg));
    atomic_can_process = !lidar_buffer.empty() && imu_buffer.back().time > lidar_buffer.front().timestamps.max;
  }
  if (atomic_can_process) {
    sync_condition_variable.notify_one();
  }
}

void ThreadedNode::lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg) {
  if (!ensure_frame_and_extrinsics(lidar_frame, lidar_msg->header.frame_id, "LiDAR")) {
    return;
  }
  try {
    const auto [timestamps, scan] = process_lidar_msg(lidar_msg);
    {
      std::lock_guard lock(buffer_mutex);
      if (imu_buffer.empty()) {
        if (lidar_buffer.size() >= max_lidar_buffer_size) {
          ROS_ERROR_STREAM_THROTTLE(5.0, "IMU buffer is empty; registration cannot start. "
                                             << "Verify imu_topic is publishing sensor_msgs/Imu "
                                             << "(current imu_topic: " << imu_topic << ").");
          return;
        }
      } else if (lidar_buffer.size() >= max_lidar_buffer_size) {
        ROS_WARN_STREAM_THROTTLE(5.0, "Registration slower than lidar rate; dropping oldest buffered scan.");
        lidar_buffer.pop();
      }
      lidar_buffer.emplace(timestamps, scan);
      atomic_can_process = !imu_buffer.empty() && imu_buffer.back().time > lidar_buffer.front().timestamps.max;
    }
    if (atomic_can_process) {
      sync_condition_variable.notify_one();
    }
  } catch (const std::invalid_argument& ex) {
    ROS_ERROR_STREAM("Encountered error, dropping frame: Error. " << ex.what());
  }
}

void ThreadedNode::registration_loop() {
  while (::ros::ok() && atomic_node_running) {
    SCOPED_PROFILER("ROS Registration Loop");
    std::unique_lock buffer_lock(buffer_mutex);
    sync_condition_variable.wait(buffer_lock, [this]() { return !atomic_node_running || atomic_can_process; });
    if (!atomic_node_running) {
      break;
    }
    if (lidar_buffer.size() > 1) {
      const size_t stale_count = lidar_buffer.size() - 1;
      ROS_WARN_STREAM_THROTTLE(5.0, "Registration backlog: skipping " << stale_count << " stale lidar scan(s).");
      while (lidar_buffer.size() > 1) {
        lidar_buffer.pop();
      }
    }
    LidarFrame frame = std::move(lidar_buffer.front());
    lidar_buffer.pop();
    registration_busy = true;
    const auto& [timestamps, scan] = frame;
    const auto& [start_stamp, end_stamp, time_vector] = timestamps;
    for (; !imu_buffer.empty() && imu_buffer.front().time < end_stamp; imu_buffer.pop()) {
      const core::ImuControl& imu_data = imu_buffer.front();
      lio->add_imu_measurement(extrinsic_imu2base, imu_data);
    }
    atomic_can_process =
        !imu_buffer.empty() && !lidar_buffer.empty() && imu_buffer.back().time > lidar_buffer.front().timestamps.max;
    buffer_lock.unlock();

    try {
      const core::Vector3dVector deskewed_frame = register_scan_locked(scan, time_vector);
      if (!deskewed_frame.empty()) {
        publish_lidar_outputs(deskewed_frame);
        publish_tf(lio->lidar_state);
      }
    } catch (const std::invalid_argument& ex) {
      ROS_ERROR_STREAM("Encountered error, dropping frame. Error: " << ex.what());
    } catch (const std::runtime_error& ex) {
      ROS_ERROR_STREAM_THROTTLE(5.0, "ICP failed, dropping frame: " << ex.what());
    }
    registration_busy = false;
  }
  atomic_node_running = false;
}

ThreadedNode::~ThreadedNode() {
  atomic_node_running = false;
  sync_condition_variable.notify_all();
  if (registration_thread.joinable()) {
    registration_thread.join();
  }
}

} // namespace rko_lio::ros
