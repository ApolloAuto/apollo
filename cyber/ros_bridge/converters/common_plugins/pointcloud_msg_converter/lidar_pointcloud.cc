/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "cyber/ros_bridge/converters/common_plugins/pointcloud_msg_converter/lidar_pointcloud.h"  // NOLINT

namespace apollo {
namespace cyber {

bool LidarPointcloud::ConvertMsg(InputTypes<InputMsgPtr>& in,
                                 OutputTypes<OutputMsgPtr>& out) {
#ifdef ENABLE_ROS_MSG
  auto in_msg = std::get<0>(in.values);
  auto out_msg = std::get<0>(out.values);
  float x, y, z;
  float intensity = -1.0;
  double timestamp = 0.0;
  auto& cloud_msg = (*in_msg);

  out_msg->set_frame_id(cloud_msg.header.frame_id);
  out_msg->set_is_dense(cloud_msg.is_dense);
  out_msg->set_width(cloud_msg.width);
  out_msg->set_height(cloud_msg.height);
  // out_msg -> set_measurement_time();

  for (size_t i = 0; i < cloud_msg.height * cloud_msg.width; ++i) {
    size_t point_offset = i * cloud_msg.point_step;

    x = ReadAsHostEndian<float>(
        cloud_msg.is_bigendian,
        &cloud_msg.data[point_offset + cloud_msg.fields[0].offset]);
    y = ReadAsHostEndian<float>(
        cloud_msg.is_bigendian,
        &cloud_msg.data[point_offset + cloud_msg.fields[1].offset]);
    z = ReadAsHostEndian<float>(
        cloud_msg.is_bigendian,
        &cloud_msg.data[point_offset + cloud_msg.fields[2].offset]);
    int intensity_index = FindFieldIndex("intensity", *in_msg);
    if (intensity_index != -1) {
      switch (cloud_msg.fields[intensity_index].datatype) {
        case sensor_msgs::msg::PointField::INT8:
        case sensor_msgs::msg::PointField::UINT8: {
          intensity = static_cast<float>(ReadAsHostEndian<uint8_t>(
              cloud_msg.is_bigendian,
              &cloud_msg.data[point_offset +
                              cloud_msg.fields[intensity_index].offset]));
          break;
        }
        case sensor_msgs::msg::PointField::INT16:
        case sensor_msgs::msg::PointField::UINT16: {
          intensity = static_cast<float>(ReadAsHostEndian<uint16_t>(
              cloud_msg.is_bigendian,
              &cloud_msg.data[point_offset +
                              cloud_msg.fields[intensity_index].offset]));
          break;
        }
        case sensor_msgs::msg::PointField::INT32:
        case sensor_msgs::msg::PointField::UINT32: {
          intensity = static_cast<float>(ReadAsHostEndian<uint32_t>(
              cloud_msg.is_bigendian,
              &cloud_msg.data[point_offset +
                              cloud_msg.fields[intensity_index].offset]));
          break;
        }
        case sensor_msgs::msg::PointField::FLOAT32: {
          intensity = static_cast<float>(ReadAsHostEndian<float>(
              cloud_msg.is_bigendian,
              &cloud_msg.data[point_offset +
                              cloud_msg.fields[intensity_index].offset]));
          break;
        }
        case sensor_msgs::msg::PointField::FLOAT64: {
          intensity = static_cast<float>(ReadAsHostEndian<double>(
              cloud_msg.is_bigendian,
              &cloud_msg.data[point_offset +
                              cloud_msg.fields[intensity_index].offset]));
          break;
        }
        default:
          throw std::runtime_error("Unsupported PointField datatype");
      }
    }

    int timestamp_index = FindFieldIndex("timestamp", *in_msg);
    if (timestamp_index != -1) {
      switch (cloud_msg.fields[timestamp_index].datatype) {
        case sensor_msgs::msg::PointField::INT8:
        case sensor_msgs::msg::PointField::UINT8: {
          timestamp = static_cast<double>(ReadAsHostEndian<uint8_t>(
              cloud_msg.is_bigendian,
              &cloud_msg.data[point_offset +
                              cloud_msg.fields[timestamp_index].offset]));
          break;
        }
        case sensor_msgs::msg::PointField::INT16:
        case sensor_msgs::msg::PointField::UINT16: {
          timestamp = static_cast<double>(ReadAsHostEndian<uint16_t>(
              cloud_msg.is_bigendian,
              &cloud_msg.data[point_offset +
                              cloud_msg.fields[timestamp_index].offset]));
          break;
        }
        case sensor_msgs::msg::PointField::INT32:
        case sensor_msgs::msg::PointField::UINT32: {
          timestamp = static_cast<double>(ReadAsHostEndian<uint32_t>(
              cloud_msg.is_bigendian,
              &cloud_msg.data[point_offset +
                              cloud_msg.fields[timestamp_index].offset]));
          break;
        }
        case sensor_msgs::msg::PointField::FLOAT32: {
          timestamp = static_cast<double>(ReadAsHostEndian<float>(
              cloud_msg.is_bigendian,
              &cloud_msg.data[point_offset +
                              cloud_msg.fields[timestamp_index].offset]));
          break;
        }
        case sensor_msgs::msg::PointField::FLOAT64: {
          timestamp = static_cast<double>(ReadAsHostEndian<double>(
              cloud_msg.is_bigendian,
              &cloud_msg.data[point_offset +
                              cloud_msg.fields[timestamp_index].offset]));
          break;
        }
        default:
          throw std::runtime_error("Unsupported PointField datatype");
      }
    }
    auto pointcloud = out_msg->add_point();
    pointcloud->set_x(x);
    pointcloud->set_y(y);
    pointcloud->set_z(z);
    pointcloud->set_intensity(static_cast<uint32_t>(std::round(intensity)));
    pointcloud->set_timestamp(static_cast<uint64_t>(timestamp * 10e9));
  }
#endif
  return true;
}

}  // namespace cyber
}  // namespace apollo
