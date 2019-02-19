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

/**
 * @file lidar_msg_transfer.h
 * @brief The class of LidarMagTransfer
 */

#pragma once

#include "modules/drivers/proto/pointcloud.pb.h"
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
  LidarMsgTransfer() = default;

  void Transfer(const drivers::PointCloud &message, LidarFrame *lidar_frame);

 protected:
  double max_height_ = 100.0;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
