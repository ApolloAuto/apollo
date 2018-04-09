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

/**
 * @file lidar_msg_transfer.h
 * @brief The class of LidarMagTransfer
 */

#ifndef MODULES_LOCALIZATION_MSF_LIDAR_MSG_TRANSFER_H_
#define MODULES_LOCALIZATION_MSF_LIDAR_MSG_TRANSFER_H_

#include "sensor_msgs/PointCloud2.h"
#include "modules/localization/msf/local_integ/localization_lidar.h"

/**
 * @namespace apollo::localization::msf
 * @brief apollo::localization::msf
 */
namespace apollo {
namespace localization {
namespace msf {

class LidarMsgTransfer {
 public:
  LidarMsgTransfer();

  static void Transfer(
      const sensor_msgs::PointCloud2 &message, LidarFrame *lidar_frame);
  void TransferCloud(
      const sensor_msgs::PointCloud2 &message, LidarFrame *lidar_frame);

 protected:
  void ParseCloudField(const sensor_msgs::PointCloud2 &message);
  void TransferOrganizedCloud32(
      const sensor_msgs::PointCloud2 &message, LidarFrame *lidar_frame);
  void TransferOrganizedCloud64(
      const sensor_msgs::PointCloud2 &message, LidarFrame *lidar_frame);
  void TransferUnorganizedCloud32(
      const sensor_msgs::PointCloud2 &message, LidarFrame *lidar_frame);
  void TransferUnorganizedCloud64(
      const sensor_msgs::PointCloud2 &message, LidarFrame *lidar_frame);

 protected:
  int width_;
  int height_;
  int x_offset_;
  int y_offset_;
  int z_offset_;
  int t_offset_;
  int i_offset_;
  int8_t x_datatype_;
  int8_t y_datatype_;
  int8_t z_datatype_;
  int x_count_;
  int y_count_;
  int z_count_;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_MSF_GNSS_MSG_TRANSFER_H_
