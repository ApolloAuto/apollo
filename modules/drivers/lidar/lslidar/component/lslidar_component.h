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

#include "modules/drivers/lidar/lslidar/proto/lslidar.pb.h"

#include "cyber/cyber.h"
#include "modules/common/util/message_util.h"
#include "modules/drivers/lidar/common/lidar_component_base.h"
#include "modules/drivers/lidar/common/sync_queue.h"
#include "modules/drivers/lidar/lslidar/driver/driver.h"
#include "modules/drivers/lidar/lslidar/parser/convert.h"

namespace apollo {
namespace drivers {
namespace lslidar {

class LslidarComponent final : public lidar::LidarComponentBase<LslidarScan> {
 public:
    virtual ~LslidarComponent();

    bool Init() override;

    void ReadScanCallback(
            const std::shared_ptr<LslidarScan> &scan_message) override;

 private:
    void DevicePollProcess();

    void ScanQueuePollProcess();

    void HandleScanFrame(const std::shared_ptr<LslidarScan> &scan_frame);

    std::shared_ptr<std::thread> device_thread_, pointcloud_convert_thread_;
    std::shared_ptr<driver::LslidarDriver>
            dvr_;  ///< driver implementation class
    std::shared_ptr<parser::Convert> converter_;
    ::apollo::lidar::drivers::SyncQueue<std::shared_ptr<LslidarScan>>
            scan_queue_;

    Config conf_;
};
CYBER_REGISTER_COMPONENT(LslidarComponent)

}  // namespace lslidar
}  // namespace drivers
}  // namespace apollo
