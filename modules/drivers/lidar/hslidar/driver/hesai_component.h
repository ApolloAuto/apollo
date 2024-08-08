/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
#include <string>

#include <hesai2/hesai_lidar_sdk.hpp>

#include "modules/drivers/lidar/hslidar/proto/hesai_config.pb.h"

#include "cyber/cyber.h"
#include "modules/drivers/lidar/common/lidar_component_base.h"

namespace apollo {
namespace drivers {
namespace lidar {

class HesaiComponent2 : public LidarComponentBase<HesaiUdpFrame> {
 public:
    virtual ~HesaiComponent2() {}
    bool Init() override;
    void ReadScanCallback(
            const std::shared_ptr<HesaiUdpFrame>& scan_message) override;

    // Used to publish point clouds through 'ros_send_point_cloud_topic'
    void SendPointCloud(const LidarDecodedFrame<LidarPointXYZIRT>& msg);
    // Used to publish the original pcake through 'ros_send_packet_topic'
    void SendPacket(const UdpFrame_t& hesai_raw_msg, double);

 private:
    HesaiConfig conf_;
    std::shared_ptr<HesaiLidarSdk<LidarPointXYZIRT>> driver_ptr_;

    int convert_threads_num_ = 1;
};

CYBER_REGISTER_COMPONENT(HesaiComponent2)

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
