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

#pragma once

#include <Eigen/Core>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <optional>
#include <ros/console.h>
#include <sophus/se3.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>

namespace rko_lio::ros::utils {
template <typename Scalar = double>
inline geometry_msgs::Pose sophus_to_pose(const Sophus::SE3<Scalar>& T) {
  geometry_msgs::Pose t;
  t.position.x = T.translation().x();
  t.position.y = T.translation().y();
  t.position.z = T.translation().z();

  Eigen::Quaternion<Scalar> q(T.so3().unit_quaternion());
  t.orientation.x = q.x();
  t.orientation.y = q.y();
  t.orientation.z = q.z();
  t.orientation.w = q.w();

  return t;
}

template <typename Scalar = double>
inline geometry_msgs::Transform sophus_to_transform(const Sophus::SE3<Scalar>& T) {
  geometry_msgs::Transform t;
  t.translation.x = T.translation().x();
  t.translation.y = T.translation().y();
  t.translation.z = T.translation().z();

  Eigen::Quaternion<Scalar> q(T.so3().unit_quaternion());
  t.rotation.x = q.x();
  t.rotation.y = q.y();
  t.rotation.z = q.z();
  t.rotation.w = q.w();

  return t;
}

template <typename Scalar = double>
inline Sophus::SE3<Scalar> transform_to_sophus(const geometry_msgs::TransformStamped& transform) {
  const auto& t = transform.transform;
  return {typename Sophus::SE3<Scalar>::QuaternionType(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z),
          typename Sophus::SE3<Scalar>::Point(t.translation.x, t.translation.y, t.translation.z)};
}

template <typename Scalar = double>
std::optional<Sophus::SE3<Scalar>>
get_transform(const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
              const std::string& from_frame,
              const std::string& to_frame,
              const std::chrono::nanoseconds time,
              const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(0)) {
  geometry_msgs::TransformStamped from_to_transform;
  const ::ros::Time tf_time = ::ros::Time().fromNSec(static_cast<uint64_t>(time.count()));
  const ::ros::Duration tf_timeout = ::ros::Duration().fromNSec(static_cast<int64_t>(timeout.count()));
  try {
    tf_buffer->_validateFrameId("from_frame", from_frame);
    tf_buffer->_validateFrameId("to frame", to_frame);
    std::string error_str;
    if (!tf_buffer->canTransform(to_frame, from_frame, tf_time, tf_timeout, &error_str)) {
      ROS_WARN_STREAM("Cannot transform from: " << from_frame << " -> to: " << to_frame
                                                 << " at time: " << time.count() << "ns because of: " << error_str);
      return std::nullopt;
    }
    from_to_transform = tf_buffer->lookupTransform(to_frame, from_frame, tf_time);
    return transform_to_sophus<Scalar>(from_to_transform);
  } catch (const tf2::InvalidArgumentException& e) {
    ROS_WARN_STREAM("TF lookup error (InvalidArgumentException): " << e.what());
    ROS_WARN_STREAM("Arguments are, to_frame: " << to_frame << ", from_frame: " << from_frame
                                                 << ", time(ns): " << time.count());
  } catch (const tf2::LookupException& e) {
    ROS_WARN_STREAM("TF lookup error (LookupException): " << e.what());
  } catch (const tf2::TransformException& ex) {
    ROS_ERROR_STREAM("Could not get the transform from: " << from_frame << " to " << to_frame << ": " << ex.what());
  }
  return std::nullopt;
}
} // namespace rko_lio::ros::utils
