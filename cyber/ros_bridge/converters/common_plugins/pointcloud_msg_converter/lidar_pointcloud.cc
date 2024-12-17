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

#ifdef ENABLE_ROS_MSG
void ParseField(sensor_msgs::msg::PointField field,
                std::shared_ptr<apollo::drivers::PointCloud> point_clouds,
                std::string field_name,
                sensor_msgs::msg::PointCloud2& raw_message) {
  switch (field.datatype) {
    case sensor_msgs::msg::PointField::INT8: {
      sensor_msgs::PointCloud2ConstIterator<int8_t> iter(raw_message,
                                                         field_name);
      int index = 0;
      for (; iter != iter.end(); ++iter) {
        if (field_name == "intensity") {
          point_clouds->mutable_point(index)->set_intensity(
              static_cast<uint32_t>(*iter));
        } else {
          point_clouds->mutable_point(index)->set_timestamp(
              static_cast<uint64_t>(*iter));
          AERROR << *iter;
        }
        index++;
      }
      break;
    }
    case sensor_msgs::msg::PointField::UINT8: {
      sensor_msgs::PointCloud2ConstIterator<int8_t> iter(raw_message,
                                                         field_name);
      int index = 0;
      for (; iter != iter.end(); ++iter) {
        if (field_name == "intensity") {
          point_clouds->mutable_point(index)->set_intensity(
              static_cast<uint32_t>(*iter));
        } else {
          point_clouds->mutable_point(index)->set_timestamp(
              static_cast<uint64_t>(*iter));
          AERROR << *iter;
        }
        index++;
      }
      break;
    }
    case sensor_msgs::msg::PointField::INT16: {
      sensor_msgs::PointCloud2ConstIterator<int16_t> iter(raw_message,
                                                          field_name);
      int index = 0;
      for (; iter != iter.end(); ++iter) {
        if (field_name == "intensity") {
          point_clouds->mutable_point(index)->set_intensity(
              static_cast<uint32_t>(*iter));
        } else {
          point_clouds->mutable_point(index)->set_timestamp(
              static_cast<uint64_t>(*iter));
          AERROR << *iter;
        }
        index++;
      }
      break;
    }
    case sensor_msgs::msg::PointField::UINT16: {
      sensor_msgs::PointCloud2ConstIterator<int16_t> iter(raw_message,
                                                          field_name);
      int index = 0;
      for (; iter != iter.end(); ++iter) {
        if (field_name == "intensity") {
          point_clouds->mutable_point(index)->set_intensity(
              static_cast<uint32_t>(*iter));
        } else {
          point_clouds->mutable_point(index)->set_timestamp(
              static_cast<uint64_t>(*iter));
          AERROR << *iter;
        }
        index++;
      }
      break;
    }
    case sensor_msgs::msg::PointField::INT32: {
      sensor_msgs::PointCloud2ConstIterator<int32_t> iter(raw_message,
                                                          field_name);
      int index = 0;
      for (; iter != iter.end(); ++iter) {
        if (field_name == "intensity") {
          point_clouds->mutable_point(index)->set_intensity(
              static_cast<uint32_t>(*iter));
        } else {
          point_clouds->mutable_point(index)->set_timestamp(
              static_cast<uint64_t>(*iter));
          AERROR << *iter;
        }
        index++;
      }
      break;
    }
    case sensor_msgs::msg::PointField::UINT32: {
      sensor_msgs::PointCloud2ConstIterator<uint32_t> iter(raw_message,
                                                           field_name);
      int index = 0;
      for (; iter != iter.end(); ++iter) {
        if (field_name == "intensity") {
          point_clouds->mutable_point(index)->set_intensity(
              static_cast<uint32_t>(*iter));
        } else {
          point_clouds->mutable_point(index)->set_timestamp(
              static_cast<uint64_t>(*iter));
          AERROR << *iter;
        }
        index++;
      }
      break;
    }
    case sensor_msgs::msg::PointField::FLOAT32: {
      sensor_msgs::PointCloud2ConstIterator<float> iter(raw_message,
                                                        field_name);
      int index = 0;
      for (; iter != iter.end(); ++iter) {
        if (field_name == "intensity") {
          point_clouds->mutable_point(index)->set_intensity(
              static_cast<uint32_t>(*iter));
        } else {
          point_clouds->mutable_point(index)->set_timestamp(
              static_cast<uint64_t>(*iter));
          AERROR << *iter;
        }
        index++;
      }
      break;
    }
    case sensor_msgs::msg::PointField::FLOAT64: {
      sensor_msgs::PointCloud2ConstIterator<double> iter(raw_message,
                                                         field_name);
      int index = 0;
      for (; iter != iter.end(); ++iter) {
        if (field_name == "intensity") {
          point_clouds->mutable_point(index)->set_intensity(
              static_cast<uint32_t>(*iter));
        } else {
          point_clouds->mutable_point(index)->set_timestamp(
              static_cast<uint64_t>(*iter));
          AERROR << *iter;
        }
        index++;
      }
      break;
    }
    default:
      throw std::runtime_error("Unsupported PointField datatype");
  }
}
#endif

bool LidarPointcloud::ConvertMsg(InputTypes<InputMsgPtr>& in,
                                 OutputTypes<OutputMsgPtr>& out) {
#ifdef ENABLE_ROS_MSG
  auto in_msg = std::get<0>(in.values);
  auto out_msg = std::get<0>(out.values);
  float x, y, z;
  float intensity = -1.0;
  double timestamp = 0.0;
  auto& cloud_msg = (*in_msg);

  bool has_x = false, has_y = false, has_z = false, has_intensity = false,
       has_time = false;
  std::string time_field_name;
  sensor_msgs::msg::PointField time_field_type;
  sensor_msgs::msg::PointField intensity_field_type;
  for (const auto& field : cloud_msg.fields) {
    if (field.name == "x") has_x = true;
    if (field.name == "y") has_y = true;
    if (field.name == "z") has_z = true;
    if (field.name == "intensity") {
      has_intensity = true;
      intensity_field_type = field;
    }
    if (field.name == "t" || field.name == "time" ||
        field.name == "timestamp") {
      has_time = true;
      time_field_type = field;
      time_field_name = field.name;
    }
  }
  if (!(has_x && has_y && has_z)) {
    AERROR << "PointCloud2 does not contain x, y, z fields.";
    return false;
  }

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_msg, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    x = *iter_x;
    y = *iter_y;
    z = *iter_z;

    auto pointcloud = out_msg->add_point();
    pointcloud->set_x(x);
    pointcloud->set_y(y);
    pointcloud->set_z(z);
  }
  if (has_time) {
    ParseField(time_field_type, out_msg, time_field_name, cloud_msg);
  }
  if (has_intensity) {
    ParseField(intensity_field_type, out_msg, "intensity", cloud_msg);
  }
#endif
  return true;
}

}  // namespace cyber
}  // namespace apollo
