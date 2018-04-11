/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/localization/msf/local_integ/lidar_msg_transfer.h"

#include "modules/common/log.h"
#include "modules/localization/common/localization_gflags.h"

namespace apollo {
namespace localization {
namespace msf {

LidarMsgTransfer::LidarMsgTransfer()
    : width_(0), height_(0), x_offset_(0), y_offset_(0), z_offset_(0),
      t_offset_(0), i_offset_(0), x_datatype_(0), y_datatype_(0),
      z_datatype_(0), x_count_(0), y_count_(0), z_count_(0) {}

void LidarMsgTransfer::Transfer(
    const sensor_msgs::PointCloud2 &message, LidarFrame *lidar_frame) {
  LidarMsgTransfer transfer;
  transfer.TransferCloud(message, lidar_frame);
  return;
}

void LidarMsgTransfer::TransferCloud(
    const sensor_msgs::PointCloud2 &lidar_data, LidarFrame *lidar_frame) {
  CHECK_NOTNULL(lidar_frame);

  ParseCloudField(lidar_data);
  // organized point cloud
  if (lidar_data.height > 1 && lidar_data.width > 1) {
    if (x_datatype_ == sensor_msgs::PointField::FLOAT32) {
      TransferOrganizedCloud32(lidar_data, lidar_frame);
    } else if (x_datatype_ == sensor_msgs::PointField::FLOAT64) {
      TransferOrganizedCloud64(lidar_data, lidar_frame);
    } else {
      AERROR << "The point cloud data type is not right!";
    }
  } else {  // unorganized point cloud
    if (x_datatype_ == sensor_msgs::PointField::FLOAT32) {
      TransferUnorganizedCloud32(lidar_data, lidar_frame);
    } else if (x_datatype_ == sensor_msgs::PointField::FLOAT64) {
      TransferUnorganizedCloud64(lidar_data, lidar_frame);
    } else {
      AERROR << "The point cloud data type is not right!";
    }
  }
  lidar_frame->measurement_time = lidar_data.header.stamp.toSec();

  if (FLAGS_lidar_debug_log_flag) {
    AINFO << std::setprecision(16)
          << "LidarMsgTransfer Debug Log: lidar msg. "
          << "[time:" << lidar_frame->measurement_time << "]"
          << "[height:" << lidar_data.height << "]"
          << "[width:" << lidar_data.width << "]"
          << "[point_step:" << lidar_data.point_step << "]"
          << "[data_type:" << x_datatype_ << "]"
          << "[point_cnt:" << x_count_ << "]"
          << "[intensity_cnt:"
          << static_cast<unsigned int>(lidar_frame->intensities.size())
          << "]";
  }
  return;
}

void LidarMsgTransfer::ParseCloudField(
    const sensor_msgs::PointCloud2 &lidar_data) {
  width_ = lidar_data.width;
  height_ = lidar_data.height;

  for (size_t i = 0; i < lidar_data.fields.size(); ++i) {
    const sensor_msgs::PointField& f = lidar_data.fields[i];
    if (f.name == "x") {
      x_offset_ = f.offset;
      x_datatype_ = f.datatype;
      x_count_ = f.count;
    } else if (f.name == "y") {
      y_offset_ = f.offset;
      y_datatype_ = f.datatype;
      y_count_ = f.count;
    } else if (f.name == "z") {
      z_offset_ = f.offset;
      z_datatype_ = f.datatype;
      z_count_ = f.count;
    } else if (f.name == "timestamp") {
      t_offset_ = f.offset;
    } else if (f.name == "intensity") {
      i_offset_ = f.offset;
    }
  }

  CHECK(x_datatype_ == y_datatype_ && y_datatype_ == z_datatype_);
  CHECK(x_datatype_ == 7 || x_datatype_ == 8);
  CHECK(x_count_ > 0 && y_count_ > 0 && z_count_ >0);

  return;
}

void LidarMsgTransfer::TransferOrganizedCloud32(
    const sensor_msgs::PointCloud2 &lidar_data, LidarFrame *lidar_frame) {
  CHECK_NOTNULL(lidar_frame);

  for (uint32_t i = 0; i < lidar_data.height; ++i) {
    for (uint32_t j = 0; j < lidar_data.width; ++j) {
      uint32_t index = i * lidar_data.width + j;
      Eigen::Vector3d pt3d;
      uint32_t offset = index * lidar_data.point_step;
      pt3d[0] = static_cast<const double>(*reinterpret_cast<const float*>(
          &lidar_data.data[offset + x_offset_]));
      pt3d[1] = static_cast<const double>(*reinterpret_cast<const float*>(
          &lidar_data.data[offset + y_offset_]));
      pt3d[2] = static_cast<const double>(*reinterpret_cast<const float*>(
          &lidar_data.data[offset + z_offset_]));
      if (!std::isnan(pt3d[0])) {
        unsigned char intensity = *reinterpret_cast<const unsigned char*>(
            &lidar_data.data[offset + i_offset_]);
        lidar_frame->pt_xs.push_back(pt3d[0]);
        lidar_frame->pt_ys.push_back(pt3d[1]);
        lidar_frame->pt_zs.push_back(pt3d[2]);
        lidar_frame->intensities.push_back(intensity);
      }
    }
  }
  return;
}

void LidarMsgTransfer::TransferOrganizedCloud64(
    const sensor_msgs::PointCloud2 &lidar_data, LidarFrame *lidar_frame) {
  CHECK_NOTNULL(lidar_frame);

  for (uint32_t i = 0; i < lidar_data.height; ++i) {
    for (uint32_t j = 0; j < lidar_data.width; ++j) {
      uint32_t index = i * lidar_data.width + j;
      Eigen::Vector3d pt3d;
      uint32_t offset = index * lidar_data.point_step;
      pt3d[0] = *reinterpret_cast<const double*>(
          &lidar_data.data[offset + x_offset_]);
      pt3d[1] = *reinterpret_cast<const double*>(
          &lidar_data.data[offset + y_offset_]);
      pt3d[2] = *reinterpret_cast<const double*>(
          &lidar_data.data[offset + z_offset_]);
      if (!std::isnan(pt3d[0])) {
        unsigned char intensity = *reinterpret_cast<const unsigned char*>(
            &lidar_data.data[offset + i_offset_]);
        lidar_frame->pt_xs.push_back(pt3d[0]);
        lidar_frame->pt_ys.push_back(pt3d[1]);
        lidar_frame->pt_zs.push_back(pt3d[2]);
        lidar_frame->intensities.push_back(intensity);
      }
    }
  }
  return;
}

void LidarMsgTransfer::TransferUnorganizedCloud32(
    const sensor_msgs::PointCloud2 &lidar_data, LidarFrame *lidar_frame) {
  CHECK_NOTNULL(lidar_frame);

  for (uint32_t i = 0; i < lidar_data.height; ++i) {
    for (uint32_t j = 0; j < lidar_data.width; ++j) {
      uint32_t index = i * lidar_data.width + j;
      Eigen::Vector3d pt3d;
      uint32_t offset = index * lidar_data.point_step;
      pt3d[0] = static_cast<const double>(*reinterpret_cast<const float*>(
          &lidar_data.data[offset + x_offset_]));
      pt3d[1] = static_cast<const double>(*reinterpret_cast<const float*>(
          &lidar_data.data[offset + y_offset_]));
      pt3d[2] = static_cast<const double>(*reinterpret_cast<const float*>(
          &lidar_data.data[offset + z_offset_]));
      if (!std::isnan(pt3d[0])) {
        unsigned char intensity = *reinterpret_cast<const unsigned char*>(
            &lidar_data.data[offset + i_offset_]);
        lidar_frame->pt_xs.push_back(pt3d[0]);
        lidar_frame->pt_ys.push_back(pt3d[1]);
        lidar_frame->pt_zs.push_back(pt3d[2]);
        lidar_frame->intensities.push_back(intensity);
      }
    }
  }
  return;
}

void LidarMsgTransfer::TransferUnorganizedCloud64(
    const sensor_msgs::PointCloud2 &lidar_data, LidarFrame *lidar_frame) {
  CHECK_NOTNULL(lidar_frame);

  for (uint32_t i = 0; i < lidar_data.height; ++i) {
    for (uint32_t j = 0; j < lidar_data.width; ++j) {
      uint32_t index = i * lidar_data.width + j;
      Eigen::Vector3d pt3d;
      uint32_t offset = index * lidar_data.point_step;
      pt3d[0] = *reinterpret_cast<const double*>(
          &lidar_data.data[offset + x_offset_]);
      pt3d[1] = *reinterpret_cast<const double*>(
          &lidar_data.data[offset + y_offset_]);
      pt3d[2] = *reinterpret_cast<const double*>(
          &lidar_data.data[offset + z_offset_]);
      if (!std::isnan(pt3d[0])) {
        unsigned char intensity = *reinterpret_cast<const unsigned char*>(
            &lidar_data.data[offset + i_offset_]);
        lidar_frame->pt_xs.push_back(pt3d[0]);
        lidar_frame->pt_ys.push_back(pt3d[1]);
        lidar_frame->pt_zs.push_back(pt3d[2]);
        lidar_frame->intensities.push_back(intensity);
      }
    }
  }
  return;
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
