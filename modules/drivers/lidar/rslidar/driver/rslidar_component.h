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

#include "rs_driver/api/lidar_driver.hpp"
#include "rs_driver/driver/driver_param.hpp"
#include "rs_driver/msg/packet.hpp"
#include "rs_driver/msg/point_cloud_msg.hpp"

#include "modules/drivers/lidar/rslidar/proto/rslidar.pb.h"
#include "modules/drivers/lidar/rslidar/proto/rslidar_config.pb.h"

#include "modules/drivers/lidar/common/lidar_component_base.h"
#include "modules/drivers/lidar/common/sync_buffering.h"
#include "modules/drivers/lidar/common/util.h"

namespace apollo {
namespace drivers {
namespace lidar {

typedef PointXYZIRT PointT;
typedef PointCloudT<PointT> PointCloudMsg;

using ::robosense::lidar::InputType;

class RslidarComponent
        : public LidarComponentBase<robosense::RobosenseScanPacket> {
 public:
    bool Init() override;

    void ReadScanCallback(const std::shared_ptr<robosense::RobosenseScanPacket>&
                                  scan_message) override;

    void RsPacketCallback(const ::robosense::lidar::Packet& lidar_packet);

    std::shared_ptr<PointCloudMsg> RsCloudAllocateCallback();

    void RsCloudPutCallback(std::shared_ptr<PointCloudMsg> rs_cloud);

    void PreparePointsMsg(PointCloud& msg);

    void ProcessCloud();

 private:
    std::shared_ptr<::robosense::lidar::LidarDriver<PointCloudMsg>> driver_ptr_;
    apollo::drivers::robosense::Config conf_;

    ::robosense::lidar::SyncQueue<std::shared_ptr<PointCloudMsg>> cloud_queue_;
    std::shared_ptr<SyncBuffering<PointCloudMsg>> cloud_buffer_;
    std::thread cloud_handle_thread_;

    int seq_ = 0;
};
CYBER_REGISTER_COMPONENT(RslidarComponent)

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
