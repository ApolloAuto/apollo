/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <limits>
#include <string>
#include "pandora_pointcloud/compensator.h"
#include "ros/this_node.h"

namespace apollo {
namespace drivers {
namespace pandora {

Compensator::Compensator(ros::NodeHandle node, ros::NodeHandle private_nh)
    : tf2_transform_listener_(tf2_buffer_, node),
      x_offset_(-1),
      y_offset_(-1),
      z_offset_(-1),
      timestamp_offset_(-1),
      timestamp_data_size_(0) {
  private_nh.param("child_frame_id", child_frame_id_,
                   std::string("hesai40"));
  private_nh.param(
      "topic_compensated_pointcloud", topic_compensated_pointcloud_,
      std::string("/apollo/sensor/pandora/hesai40/compensator/PointCloud2"));
  private_nh.param("topic_pointcloud", topic_pointcloud_,
                   std::string("/apollo/sensor/pandora/hesai40/PointCloud2"));
  private_nh.param("queue_size", queue_size_, 10);
  private_nh.param("tf_query_timeout", tf_timeout_, 0.1f);

  // advertise output point cloud (before subscribing to input data)
  compensation_pub_ = node.advertise<sensor_msgs::PointCloud2>(
      topic_compensated_pointcloud_, queue_size_);
  pointcloud_sub_ =
      node.subscribe(topic_pointcloud_, queue_size_,
                     &Compensator::pointcloud_callback,
                     reinterpret_cast<Compensator*>(this));
}

void Compensator::pointcloud_callback(
    const sensor_msgs::PointCloud2ConstPtr& msg) {
  if (!check_message(msg)) {
    ROS_FATAL("MotionCompensation : Input point cloud data field is invalid");
    return;
  }

  Eigen::Affine3d pose_min_time;
  Eigen::Affine3d pose_max_time;

  double timestamp_min = 0;
  double timestamp_max = 0;
  get_timestamp_interval(msg, &timestamp_min, &timestamp_max);

  // compensate point cloud, remove nan point
  if (query_pose_affine_from_tf2(timestamp_min, &pose_min_time) &&
      query_pose_affine_from_tf2(timestamp_max, &pose_max_time)) {
    // we change message after motion compesation
    sensor_msgs::PointCloud2::Ptr q_msg(new sensor_msgs::PointCloud2());
    *q_msg = *msg;
    motion_compensation<float>(q_msg, timestamp_min, timestamp_max,
                               pose_min_time, pose_max_time);
    q_msg->header.stamp.fromSec(timestamp_max);
    compensation_pub_.publish(q_msg);
  }
}

inline void Compensator::get_timestamp_interval(
    const sensor_msgs::PointCloud2ConstPtr& msg, double* timestamp_min,
    double* timestamp_max) {
  *timestamp_max = 0.0;
  *timestamp_min = std::numeric_limits<double>::max();
  int total = msg->width * msg->height;

  // get min time and max time
  for (int i = 0; i < total; ++i) {
    double timestamp = 0.0;
    memcpy(&timestamp, &msg->data[i * msg->point_step + timestamp_offset_],
           timestamp_data_size_);

    if (timestamp < *timestamp_min) {
      *timestamp_min = timestamp;
    }
    if (timestamp > *timestamp_max) {
      *timestamp_max = timestamp;
    }
  }
}

// TODO(a): if point type is always float, and timestamp is always double?
inline bool Compensator::check_message(
    const sensor_msgs::PointCloud2ConstPtr& msg) {
  // check msg width and height
  if (msg->width == 0 || msg->height == 0) {
    return false;
  }

  int x_data_type = 0;
  int y_data_type = 0;
  int z_data_type = 0;

  // TODO(a): will use a new datastruct with interface to get offset,
  // datatype,datasize...
  for (size_t i = 0; i < msg->fields.size(); ++i) {
    const sensor_msgs::PointField& f = msg->fields[i];

    if (f.name == "x") {
      x_offset_ = f.offset;
      x_data_type = f.datatype;
      if ((x_data_type != 7 && x_data_type != 8) || f.count != 1 ||
          x_offset_ == -1) {
        return false;
      }
    } else if (f.name == "y") {
      y_offset_ = f.offset;
      y_data_type = f.datatype;
      if (f.count != 1 || y_offset_ == -1) {
        return false;
      }
    } else if (f.name == "z") {
      z_offset_ = f.offset;
      z_data_type = f.datatype;
      if (f.count != 1 || z_offset_ == -1) {
        return false;
      }
    } else if (f.name == "timestamp") {
      timestamp_offset_ = f.offset;
      timestamp_data_size_ = f.count * get_field_size(f.datatype);
      if (timestamp_offset_ == -1 || timestamp_data_size_ == -1) {
        return false;
      }
    } else {
      ROS_DEBUG_STREAM("get a unused field name:" << f.name);
    }
  }

  // check offset if valid
  if (x_offset_ == -1 || y_offset_ == -1 || z_offset_ == -1 ||
      timestamp_offset_ == -1 || timestamp_data_size_ == -1) {
    return false;
  }
  if (!(x_data_type == y_data_type && y_data_type == z_data_type)) {
    return false;
  }
  return true;
}

bool Compensator::query_pose_affine_from_tf2(const double& timestamp,
                                             Eigen::Affine3d* pose) {
  ros::Time query_time(timestamp);
  std::string err_string;
  if (!tf2_buffer_.canTransform("world", child_frame_id_, query_time,
                                ros::Duration(tf_timeout_), &err_string)) {
    ROS_WARN_STREAM("Can not find transform. "
                    << std::fixed << timestamp
                    << " Error info: " << err_string);
    return false;
  }

  geometry_msgs::TransformStamped stamped_transform;

  try {
    stamped_transform =
        tf2_buffer_.lookupTransform("world", child_frame_id_, query_time);
  } catch (tf2::TransformException& ex) {
    ROS_ERROR_STREAM(ex.what());
    return false;
  }

  tf::transformMsgToEigen(stamped_transform.transform, *pose);
  // ROS_DEBUG_STREAM("pose matrix : " << pose);
  return true;
}

/** \brief Obtains the size of a specific field data type in bytes
 * \param[in] datatype the field data type
 */
inline uint Compensator::get_field_size(const int datatype) {
  switch (datatype) {
    case sensor_msgs::PointField::INT8:
    case sensor_msgs::PointField::UINT8:
      return 1;

    case sensor_msgs::PointField::INT16:
    case sensor_msgs::PointField::UINT16:
      return 2;

    case sensor_msgs::PointField::INT32:
    case sensor_msgs::PointField::UINT32:
    case sensor_msgs::PointField::FLOAT32:
      return 4;

    case sensor_msgs::PointField::FLOAT64:
      return 8;

    default:
      ROS_ERROR_STREAM("can not get field size by datatype:" << datatype);
      return 0;
  }
}

template <typename Scalar>
void Compensator::motion_compensation(const sensor_msgs::PointCloud2::Ptr& msg,
                                      const double timestamp_min,
                                      const double timestamp_max,
                                      const Eigen::Affine3d& pose_min_time,
                                      const Eigen::Affine3d& pose_max_time) {
  using std::abs;
  using std::sin;
  using std::acos;

  Eigen::Vector3d translation =
      pose_min_time.translation() - pose_max_time.translation();
  Eigen::Quaterniond q_max(pose_max_time.linear());
  Eigen::Quaterniond q_min(pose_min_time.linear());
  Eigen::Quaterniond q1(q_max.conjugate() * q_min);
  Eigen::Quaterniond q0(Eigen::Quaterniond::Identity());
  q1.normalize();
  translation = q_max.conjugate() * translation;

  int total = msg->width * msg->height;

  double d = q0.dot(q1);
  double abs_d = abs(d);
  double f = 1.0 / (timestamp_max - timestamp_min);

  // Threshold for a "significant" rotation from min_time to max_time:
  // The LiDAR range accuracy is ~2 cm. Over 70 meters range, it means an angle
  // of 0.02 / 70 =
  // 0.0003 rad. So, we consider a rotation "significant" only if the scalar
  // part of quaternion is
  // less than cos(0.0003 / 2) = 1 - 1e-8.
  if (abs_d < 1.0 - 1.0e-8) {
    double theta = acos(abs_d);
    double sin_theta = sin(theta);
    double c1_sign = (d > 0) ? 1 : -1;
    for (int i = 0; i < total; ++i) {
      size_t offset = i * msg->point_step;
      Scalar* x_scalar =
          reinterpret_cast<Scalar*>(&msg->data[offset + x_offset_]);
      if (std::isnan(*x_scalar)) {
        ROS_DEBUG_STREAM("nan point do not need motion compensation");
        continue;
      }
      Scalar* y_scalar =
          reinterpret_cast<Scalar*>(&msg->data[offset + y_offset_]);
      Scalar* z_scalar =
          reinterpret_cast<Scalar*>(&msg->data[offset + z_offset_]);
      Eigen::Vector3d p(*x_scalar, *y_scalar, *z_scalar);

      double tp = 0.0;
      memcpy(&tp, &msg->data[i * msg->point_step + timestamp_offset_],
             timestamp_data_size_);
      double t = (timestamp_max - tp) * f;

      Eigen::Translation3d ti(t * translation);

      double c0 = sin((1 - t) * theta) / sin_theta;
      double c1 = sin(t * theta) / sin_theta * c1_sign;
      Eigen::Quaterniond qi(c0 * q0.coeffs() + c1 * q1.coeffs());

      Eigen::Affine3d trans = ti * qi;
      p = trans * p;
      *x_scalar = p.x();
      *y_scalar = p.y();
      *z_scalar = p.z();
    }
    return;
  }
  // Not a "significant" rotation. Do translation only.
  for (int i = 0; i < total; ++i) {
    Scalar* x_scalar =
        reinterpret_cast<Scalar*>(&msg->data[i * msg->point_step + x_offset_]);
    if (std::isnan(*x_scalar)) {
      ROS_DEBUG_STREAM("nan point do not need motion compensation");
      continue;
    }
    Scalar* y_scalar =
        reinterpret_cast<Scalar*>(&msg->data[i * msg->point_step + y_offset_]);
    Scalar* z_scalar =
        reinterpret_cast<Scalar*>(&msg->data[i * msg->point_step + z_offset_]);
    Eigen::Vector3d p(*x_scalar, *y_scalar, *z_scalar);

    double tp = 0.0;
    memcpy(&tp, &msg->data[i * msg->point_step + timestamp_offset_],
           timestamp_data_size_);
    double t = (timestamp_max - tp) * f;
    Eigen::Translation3d ti(t * translation);

    p = ti * p;
    *x_scalar = p.x();
    *y_scalar = p.y();
    *z_scalar = p.z();
  }
}

}  // namespace pandora
}  // namespace drivers
}  // namespace apollo
