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

#include <memory>

#include "modules/drivers/lidar/seyond/src/seyond_driver.h"
#include "modules/drivers/lidar/seyond/proto/seyond.pb.h"
#include "modules/drivers/lidar/seyond/proto/seyond_config.pb.h"

#include "modules/drivers/lidar/common/lidar_component_base.h"
#include "modules/drivers/lidar/common/sync_buffering.h"
#include "modules/drivers/lidar/common/util.h"

namespace apollo {
namespace drivers {
namespace lidar {

class SeyondComponent
    : public LidarComponentBase<seyond::SeyondScan> {
 public:
  bool Init() override;

  void ReadScanCallback(
      const std::shared_ptr<seyond::SeyondScan>& scan_message) override;

  void PointCloudCallback();

  void SeyondPacketCallback(const InnoDataPacket *pkt, bool is_next_frame);


 private:
  std::shared_ptr<SeyondDriver> driver_ptr_;
  apollo::drivers::seyond::Config conf_;

  std::shared_ptr<seyond::SeyondScan> scan_packets_ptr_{nullptr};

  uint32_t table_send_hz_{10};
  uint32_t frame_count_{0};
};
CYBER_REGISTER_COMPONENT(SeyondComponent)

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
