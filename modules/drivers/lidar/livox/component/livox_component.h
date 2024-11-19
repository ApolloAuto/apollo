/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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
#pragma once

#include <deque>
#include <memory>

#include <boost/format.hpp>

#include "livox_lidar_api.h"  //NOLINT
#include "livox_lidar_def.h"  //NOLINT

#include "modules/drivers/lidar/livox/proto/livox.pb.h"

#include "modules/drivers/lidar/common/lidar_component_base.h"
#include "modules/drivers/lidar/common/util.h"
#include "modules/drivers/lidar/livox/component/livox_obserable_binder.h"
#include "modules/drivers/lidar/livox/driver/livox_util.h"

namespace apollo {
namespace drivers {
namespace lidar {

class LivoxLidarComponent final : public LidarComponentBase<livox::LivoxScan> {
 public:
  void BinaryDataProcess(const unsigned char* data, const int& data_type,
                         const int& point_size, const uint64_t& pkt_timestamp,
                         const uint32_t& time_interval);

  void PointCloudCallback(uint32_t handle, const uint8_t dev_type,
                          LivoxLidarEthernetPacket* data, void* client_data);

  size_t GetEthPacketByteSize(LivoxLidarEthernetPacket* data);

  void PreparePointsMsg(PointCloud& msg);

  bool Init() override;

  void ReadScanCallback(
      const std::shared_ptr<livox::LivoxScan>& scan_message) override;

  void CheckTimestampAndPublishPointCloud();

  livox::Config config_;
  std::deque<PointXYZIT> integral_queue_;

  uint64_t last_pointcloud_pub_timestamp_{0};
  double pointcloud_freq_ = {10.0};  // Hz
  double integral_time_ = {0.1};     // second
};
CYBER_REGISTER_COMPONENT(LivoxLidarComponent)

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
