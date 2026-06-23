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
#include "rko_lio/ros/utils/utils.hpp"
// other
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <stdexcept>

namespace rko_lio::core {
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(LIO::Config,
                                   deskew,
                                   max_iterations,
                                   voxel_size,
                                   max_points_per_voxel,
                                   max_range,
                                   min_range,
                                   convergence_criterion,
                                   max_correspondence_distance,
                                   max_num_threads,
                                   initialization_phase,
                                   max_expected_jerk,
                                   double_downsample,
                                   min_beta)
} // namespace rko_lio::core

namespace rko_lio::ros {

core::ImuControl imu_msg_to_imu_data(const sensor_msgs::Imu& imu_msg) {
  core::ImuControl imu_data;
  imu_data.time = utils::to_ns(imu_msg.header.stamp);
  imu_data.angular_velocity = utils::ros_xyz_to_eigen_vector3d(imu_msg.angular_velocity);
  imu_data.acceleration = utils::ros_xyz_to_eigen_vector3d(imu_msg.linear_acceleration);
  return imu_data;
}

BaseNode::BaseNode(const std::string& node_name)
    : nh_(), pnh_("~") {
  pnh_.param<std::string>("imu_topic", imu_topic, "");
  if (imu_topic.empty()) {
    throw std::runtime_error("Required parameter 'imu_topic' is not set");
  }
  pnh_.param<std::string>("lidar_topic", lidar_topic, "");
  if (lidar_topic.empty()) {
    throw std::runtime_error("Required parameter 'lidar_topic' is not set");
  }
  pnh_.param<std::string>("base_frame", base_frame, "");
  if (base_frame.empty()) {
    throw std::runtime_error("Required parameter 'base_frame' is not set");
  }
  pnh_.param<std::string>("imu_frame", imu_frame, imu_frame);
  pnh_.param<std::string>("lidar_frame", lidar_frame, lidar_frame);
  pnh_.param<std::string>("odom_frame", odom_frame, odom_frame);
  pnh_.param<std::string>("odom_topic", odom_topic, odom_topic);

  // tf
  pnh_.param<bool>("invert_odom_tf", invert_odom_tf, invert_odom_tf);
  tf_buffer = std::make_shared<tf2_ros::Buffer>();
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>();

  // publishing
  odom_publisher = nh_.advertise<nav_msgs::Odometry>(odom_topic, 1);

  pnh_.param<bool>("publish_lidar_acceleration", publish_lidar_acceleration, publish_lidar_acceleration);
  if (publish_lidar_acceleration) {
    lidar_accel_publisher = nh_.advertise<geometry_msgs::AccelStamped>("rko_lio/lidar_acceleration", 1);
  }

  pnh_.param<bool>("publish_deskewed_scan", publish_deskewed_scan, publish_deskewed_scan);
  if (publish_deskewed_scan) {
    pnh_.param<std::string>("deskewed_scan_topic", deskewed_scan_topic, deskewed_scan_topic);
    frame_publisher = nh_.advertise<sensor_msgs::PointCloud2>(deskewed_scan_topic, 1);
  }

  pnh_.param<bool>("publish_local_map", publish_local_map, publish_local_map);
  if (publish_local_map) {
    pnh_.param<std::string>("map_topic", map_topic, map_topic);
    double publish_map_after_seconds = core::to_seconds(publish_map_after);
    pnh_.param<double>("publish_map_after", publish_map_after_seconds, publish_map_after_seconds);
    publish_map_after = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(publish_map_after_seconds));
    map_publisher = nh_.advertise<sensor_msgs::PointCloud2>(map_topic, 1);
    map_publish_thread = std::thread([this]() { publish_map_loop(); });
  }

  // lio params
  core::LIO::Config lio_config{};
  pnh_.param<bool>("deskew", lio_config.deskew, lio_config.deskew);
  {
    int max_iterations_int = static_cast<int>(lio_config.max_iterations);
    pnh_.param<int>("max_iterations", max_iterations_int, max_iterations_int);
    lio_config.max_iterations = static_cast<size_t>(max_iterations_int);
  }
  pnh_.param<double>("voxel_size", lio_config.voxel_size, lio_config.voxel_size);
  pnh_.param<int>("max_points_per_voxel", lio_config.max_points_per_voxel, lio_config.max_points_per_voxel);
  pnh_.param<double>("max_range", lio_config.max_range, lio_config.max_range);
  pnh_.param<double>("min_range", lio_config.min_range, lio_config.min_range);
  pnh_.param<double>("convergence_criterion", lio_config.convergence_criterion, lio_config.convergence_criterion);
  pnh_.param<double>("max_correspondence_distance", lio_config.max_correspondence_distance,
                     lio_config.max_correspondence_distance);
  pnh_.param<int>("max_num_threads", lio_config.max_num_threads, lio_config.max_num_threads);
  pnh_.param<bool>("initialization_phase", lio_config.initialization_phase, lio_config.initialization_phase);
  pnh_.param<double>("max_expected_jerk", lio_config.max_expected_jerk, lio_config.max_expected_jerk);
  pnh_.param<bool>("double_downsample", lio_config.double_downsample, lio_config.double_downsample);
  pnh_.param<double>("min_beta", lio_config.min_beta, lio_config.min_beta);
  lio = std::make_unique<core::LIO>(lio_config);

  // Lidar per-point timestamp processing params
  pnh_.param<double>("lidar_timestamps/multiplier_to_seconds",
                     timestamp_proc_config.multiplier_to_seconds,
                     timestamp_proc_config.multiplier_to_seconds);
  pnh_.param<bool>("lidar_timestamps/force_absolute",
                   timestamp_proc_config.force_absolute,
                   timestamp_proc_config.force_absolute);
  pnh_.param<bool>("lidar_timestamps/force_relative",
                   timestamp_proc_config.force_relative,
                   timestamp_proc_config.force_relative);

  parse_cli_extrinsics();

  ROS_INFO_STREAM("Subscribed to IMU: "
                      << imu_topic << (!imu_frame.empty() ? " (frame " + imu_frame + ")" : "") << " and LiDAR: "
                      << lidar_topic << (!lidar_frame.empty() ? " (frame " + lidar_frame + ")" : "")
                      << ". Max number of threads: " << lio_config.max_num_threads << ". Publishing odometry to "
                      << odom_topic << " ( " << odom_frame
                      << " ) and acceleration "
                         "estimates to rko_lio/lidar_acceleration. Deskewing is "
                      << (lio->config.deskew ? "enabled" : "disabled") << "."
                      << (publish_deskewed_scan ? (" Publishing deskewed_cloud to " + deskewed_scan_topic + ".")
                                                : ""));

  // disk logging
  pnh_.param<bool>("dump_results", dump_results, dump_results);
  pnh_.param<std::string>("results_dir", results_dir, results_dir);
  pnh_.param<std::string>("run_name", run_name, run_name);

  ROS_INFO("RKO LIO Node is up!");
}

void BaseNode::parse_cli_extrinsics() {
  auto parse_extrinsic = [this](const std::string& name, Sophus::SE3d& extrinsic) {
    const std::string param_name = "extrinsic_" + name + "2base_quat_xyzw_xyz";
    std::vector<double> vec;
    pnh_.param<std::vector<double>>(param_name, vec, std::vector<double>{});

    if (vec.size() != 7) {
      if (!vec.empty()) {
        ROS_WARN_STREAM("Parameter 'extrinsic_"
                            << name << "2base_quat_xyzw_xyz' is set but has wrong size: " << vec.size()
                            << ". Expected 7 (qx, qy, qz, qw, x, y, z). check the value: "
                            << Eigen::Map<const Eigen::VectorXd>(vec.data(), vec.size()).transpose());
      }
      return false;
    }
    Eigen::Quaterniond q(vec[3], vec[0], vec[1], vec[2]);
    if (q.norm() < 1e-6) {
      throw std::runtime_error(name + " extrinsic quaternion has zero norm");
    }
    extrinsic = Sophus::SE3d(q, Eigen::Vector3d(vec[4], vec[5], vec[6]));
    ROS_INFO_STREAM("Parsed " << name << " extrinsic as: " << extrinsic.log().transpose());
    return true;
  };
  const bool imu_ok = parse_extrinsic("imu", extrinsic_imu2base);
  const bool lidar_ok = parse_extrinsic("lidar", extrinsic_lidar2base);
  extrinsics_set = imu_ok && lidar_ok;
}

bool BaseNode::ensure_frame_and_extrinsics(std::string& target_frame,
                                           const std::string& msg_frame,
                                           std::string_view kind) {
  if (target_frame.empty()) {
    if (msg_frame.empty() && !extrinsics_set) {
      throw std::runtime_error(std::string(kind) +
                               " message header has no frame id and we need it to query TF for the extrinsics. "
                               "Either specify the frame id or the extrinsic manually.");
    }
    target_frame = msg_frame;
    ROS_INFO_STREAM("Parsed the " << kind << " frame id as: " << target_frame);
  }
  return check_and_set_extrinsics();
}

bool BaseNode::check_and_set_extrinsics() {
  if (extrinsics_set) {
    return true;
  }
  using namespace std::chrono_literals;
  const std::optional<Sophus::SE3d> imu_transform = utils::get_transform(tf_buffer, imu_frame, base_frame, 0s);
  if (!imu_transform) {
    return false;
  }
  const std::optional<Sophus::SE3d> lidar_transform = utils::get_transform(tf_buffer, lidar_frame, base_frame, 0s);
  if (!lidar_transform) {
    return false;
  }
  extrinsic_imu2base = imu_transform.value();
  extrinsic_lidar2base = lidar_transform.value();
  extrinsics_set = true;
  return true;
}

std::tuple<core::Timestamps, core::Vector3dVector>
BaseNode::process_lidar_msg(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg) const {
  const core::Nsec header_stamp = utils::to_ns(lidar_msg->header.stamp);
  if (lio->config.deskew) {
    const auto& [scan, raw_timestamps] = utils::point_cloud2_to_eigen_with_timestamps(lidar_msg);
    const core::Timestamps& timestamps = core::process_timestamps(raw_timestamps, header_stamp, timestamp_proc_config);
    return {timestamps, scan};
  }
  ROS_WARN_STREAM_ONCE("Deskewing is disabled. Populating timestamps with static header time.");
  const core::Vector3dVector scan = utils::point_cloud2_to_eigen(lidar_msg);
  return {{.min = header_stamp, .max = header_stamp, .times = core::TimestampVector(scan.size(), header_stamp)}, scan};
}

core::Vector3dVector BaseNode::register_scan_locked(const core::Vector3dVector& scan,
                                                    const core::TimestampVector& time_vector) {
  if (publish_local_map) {
    std::lock_guard lock(local_map_mutex);
    return lio->register_scan(extrinsic_lidar2base, scan, time_vector);
  }
  return lio->register_scan(extrinsic_lidar2base, scan, time_vector);
}

void BaseNode::publish_lidar_outputs(const core::Vector3dVector& deskewed_frame) const {
  if (publish_deskewed_scan) {
    std_msgs::Header header;
    header.frame_id = lidar_frame;
    header.stamp = utils::to_ros_time(lio->lidar_state.time);
    frame_publisher.publish(utils::eigen_to_point_cloud2(deskewed_frame, header));
  }
  publish_odometry(lio->lidar_state, odom_publisher);
  if (publish_lidar_acceleration) {
    publish_lidar_accel(lio->lidar_state);
  }
}

void BaseNode::publish_odometry(const core::State& state, const ::ros::Publisher& publisher) const {
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = utils::to_ros_time(state.time);
  odom_msg.header.frame_id = odom_frame;
  odom_msg.child_frame_id = base_frame;
  odom_msg.pose.pose = utils::sophus_to_pose(state.pose);
  utils::eigen_vector3d_to_ros_xyz(state.velocity, odom_msg.twist.twist.linear);
  utils::eigen_vector3d_to_ros_xyz(state.angular_velocity, odom_msg.twist.twist.angular);
  publisher.publish(odom_msg);
}

void BaseNode::publish_tf(const core::State& state) const {
  geometry_msgs::TransformStamped transform_msg;
  transform_msg.header.stamp = utils::to_ros_time(state.time);
  if (invert_odom_tf) {
    transform_msg.header.frame_id = base_frame;
    transform_msg.child_frame_id = odom_frame;
    transform_msg.transform = utils::sophus_to_transform(state.pose.inverse());
  } else {
    transform_msg.header.frame_id = odom_frame;
    transform_msg.child_frame_id = base_frame;
    transform_msg.transform = utils::sophus_to_transform(state.pose);
  }
  tf_broadcaster->sendTransform(transform_msg);
}

void BaseNode::publish_lidar_accel(const core::State& state) const {
  geometry_msgs::AccelStamped accel_msg;
  accel_msg.header.stamp = utils::to_ros_time(state.time);
  accel_msg.header.frame_id = base_frame;
  utils::eigen_vector3d_to_ros_xyz(state.linear_acceleration, accel_msg.accel.linear);
  lidar_accel_publisher.publish(accel_msg);
}

void BaseNode::publish_map_loop() {
  while (atomic_node_running) {
    std::this_thread::sleep_for(publish_map_after);
    std::unique_lock lock(local_map_mutex);
    if (lio->map.empty()) {
      ROS_WARN_ONCE("Local map publish thread: Local map is empty.");
      continue;
    }
    const core::Vector3dVector map_points = lio->map.pointcloud();
    lock.unlock();
    std_msgs::Header map_header;
    map_header.stamp = ::ros::Time::now();
    map_header.frame_id = odom_frame;
    map_publisher.publish(utils::eigen_to_point_cloud2(map_points, map_header));
  }
}

BaseNode::~BaseNode() {
  atomic_node_running = false;
  if (dump_results) {
    dump_results_to_disk(results_dir, run_name);
  }
  if (map_publish_thread.joinable()) {
    map_publish_thread.join();
  }
}

void BaseNode::dump_results_to_disk(const std::filesystem::path& results_dir, const std::string& run_name) const {
  try {
    std::filesystem::create_directories(results_dir);
    int index = 0;
    std::filesystem::path output_dir = results_dir / (run_name + "_" + std::to_string(index));
    while (std::filesystem::exists(output_dir)) {
      ++index;
      output_dir = results_dir / (run_name + "_" + std::to_string(index));
    }
    std::filesystem::create_directory(output_dir);
    const std::filesystem::path output_file = output_dir / (run_name + "_tum_" + std::to_string(index) + ".txt");
    if (std::ofstream file(output_file); file.is_open()) {
      for (const auto& [timestamp, pose] : lio->poses_with_timestamps) {
        const Eigen::Vector3d& translation = pose.translation();
        const Eigen::Quaterniond& quaternion = pose.so3().unit_quaternion();
        file << std::fixed << std::setprecision(6) << core::to_seconds(timestamp) << " " << translation.x() << " "
             << translation.y() << " " << translation.z() << " " << quaternion.x() << " " << quaternion.y() << " "
             << quaternion.z() << " " << quaternion.w() << "\n";
      }
      std::cout << "Poses written to " << std::filesystem::absolute(output_file) << "\n";
    }
    const nlohmann::json json_config = {{"config", lio->config}};
    const std::filesystem::path config_file = output_dir / "config.json";
    if (std::ofstream file(config_file); file.is_open()) {
      file << json_config.dump(4);
      std::cout << "Configuration written to " << config_file << "\n";
    }
  } catch (const std::filesystem::filesystem_error& ex) {
    std::cerr << "[WARNING] Cannot write files to disk, encountered filesystem error: " << ex.what() << "\n";
  }
}

} // namespace rko_lio::ros
