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

#pragma once

#include <atomic>
#include <future>
#include <memory>
#include <vector>

#include "cyber/cyber.h"
#include "modules/common_msgs/sensor_msgs/oculii_radar.pb.h"
#include "modules/drivers/radar/oculii_radar/proto/oculii_radar_conf.pb.h"
#include "modules/drivers/radar/oculii_radar/parser/oculii_radar_udp_parser.h"

namespace apollo {
namespace drivers {
namespace radar {

using apollo::cyber::Component;
using apollo::cyber::Writer;
using apollo::drivers::OculiiPointCloud;
using apollo::drivers::oculii_radar::OculiiRadarConf;

class OculiiRadarComponent : public Component<> {
 public:
  /**
   * @brief initialize OculiiRadarComponent
   * @return bool initialization status
   */
  bool Init() override;
  ~OculiiRadarComponent();

 private:
  /**
   * @brief thread to parse the udp packet from radar
   */
  void run();

  std::shared_ptr<Writer<OculiiPointCloud>> writer_ = nullptr;
  std::unique_ptr<OculiiRadarUdpParser> parser_;
  std::shared_ptr<OculiiRadarConf> config_;

  std::future<void> async_result_;
  std::atomic<bool> running_ = {false};

  float frame_drop_interval_;
  uint64_t last_process_time_;
};

CYBER_REGISTER_COMPONENT(OculiiRadarComponent)
}  // namespace radar
}  // namespace drivers
}  // namespace apollo
