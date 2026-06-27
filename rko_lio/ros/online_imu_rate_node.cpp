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

#include "base_node.hpp"
#include "rko_lio/core/process_timestamps.hpp"
#include "rko_lio/core/profiler.hpp"
#include "rko_lio/ros/utils/utils.hpp"
// ros
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
// other
#include <stdexcept>

namespace rko_lio::ros {

class OnlineImuRateNode : public BaseNode {
public:
  ::ros::Publisher odom_at_imu_rate_publisher;
  ::ros::Subscriber imu_sub;
  ::ros::Subscriber lidar_sub;
  core::Timer timer;

  std::string odom_at_imu_rate_topic = "rko_lio/odom_at_imu_rate";
  bool tf_at_imu_rate = false;

  OnlineImuRateNode(const OnlineImuRateNode&) = delete;
  OnlineImuRateNode(OnlineImuRateNode&&) = delete;
  OnlineImuRateNode& operator=(const OnlineImuRateNode&) = delete;
  OnlineImuRateNode& operator=(OnlineImuRateNode&&) = delete;

  OnlineImuRateNode() : BaseNode("rko_lio_online_imu_rate_node"), timer("RKO LIO Online IMU-rate Node") {
    pnh.param<std::string>("seq/odom_at_imu_rate_topic", odom_at_imu_rate_topic, odom_at_imu_rate_topic);
    pnh.param<bool>("seq/tf_at_imu_rate", tf_at_imu_rate, tf_at_imu_rate);

    odom_at_imu_rate_publisher = nh.advertise<nav_msgs::Odometry>(odom_at_imu_rate_topic, 1);

    ROS_INFO_STREAM("OnlineImuRateNode publishing IMU-rate odometry to "
                    << odom_at_imu_rate_topic << ", TF rate: " << (tf_at_imu_rate ? "imu" : "lidar"));

    imu_sub = nh.subscribe<sensor_msgs::Imu>(
        imu_topic, 100, [this](const sensor_msgs::Imu::ConstPtr& msg) { imu_callback(msg); });

    lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>(
        lidar_topic, 10, [this](const sensor_msgs::PointCloud2::ConstPtr& msg) { lidar_callback(msg); });
  }

  void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    if (!ensure_frame_and_extrinsics(imu_frame, imu_msg->header.frame_id, "IMU")) {
      return;
    }

    const core::ImuControl imu_data = imu_msg_to_imu_data(*imu_msg);
    lio->add_imu_measurement(extrinsic_imu2base, imu_data);

    if (!(lio->imu_state.time > core::Nsec{0})) {
      return;
    }
    publish_odometry(lio->imu_state, odom_at_imu_rate_publisher);
    if (tf_at_imu_rate) {
      publish_tf(lio->imu_state);
    }
  }

  void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg) {
    if (!ensure_frame_and_extrinsics(lidar_frame, lidar_msg->header.frame_id, "LiDAR")) {
      return;
    }

    try {
      const auto [timestamps, scan] = process_lidar_msg(lidar_msg);
      const core::Vector3dVector deskewed_frame = register_scan_locked(scan, timestamps.times);
      if (deskewed_frame.empty()) {
        return;
      }
      publish_lidar_outputs(deskewed_frame);
      publish_tf(lio->lidar_state);
    } catch (const std::invalid_argument& ex) {
      ROS_ERROR_STREAM("Encountered error, dropping frame. Error: " << ex.what());
    }
  }
};

} // namespace rko_lio::ros

int main(int argc, char** argv) {
  ::ros::init(argc, argv, "rko_lio_online_imu_rate_node");
  rko_lio::ros::OnlineImuRateNode node;
  ::ros::spin();
  return 0;
}
