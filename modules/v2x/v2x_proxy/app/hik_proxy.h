/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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
 * @file v2x_proxy.h
 * @brief define v2x proxy class
 */

#pragma once

#include <atomic>
#include <memory>
#include <set>
#include <string>
#include <thread>

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/v2x/proto/spat_data.pb.h"

#include "cyber/cyber.h"
#include "modules/bridge/common/udp_listener.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/map/hdmap/adapter/coordinate_convert_tool.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/v2x/common/json_parse.h"
#include "modules/v2x/common/v2x_proxy_gflags.h"
#include "modules/v2x/v2x_proxy/app/proxy.h"
#include "modules/v2x/v2x_proxy/proto_adapter/proto_adapter.h"

using apollo::bridge::UDPListener;
using apollo::planning::ADCTrajectory;

namespace apollo {
namespace v2x {

class HikProxy : public Proxy {
 public:
  HikProxy();

  ~HikProxy();

  void stop() {
    exit_ = true;
    v2x_car_status_timer_->Stop();
  }

 private:
  std::unique_ptr<::apollo::cyber::Timer> v2x_car_status_timer_;
  std::unique_ptr<::apollo::cyber::Node> node_ = nullptr;
  std::unique_ptr<std::thread> recv_thread_ = nullptr;
  std::shared_ptr<::apollo::cyber::Reader<::apollo::canbus::Chassis>>
      canbus_reader_ = nullptr;
  std::shared_ptr<::apollo::cyber::Reader<ADCTrajectory>> planning_reader_ =
      nullptr;
  std::shared_ptr<
      ::apollo::cyber::Reader<::apollo::localization::LocalizationEstimate>>
      localization_reader_ = nullptr;
  std::shared_ptr<
      ::apollo::cyber::Writer<::apollo::perception::TrafficLightDetection>>
      udp_traffic_light_writer_;
  std::mutex mutex_canbus_;
  std::mutex mutex_localization_;
  std::string to_coordinate_ = "+proj=longlat +datum=WGS84 +no_defs";
  std::string from_coordinate_ =
      absl::StrCat("+proj=utm +zone=", std::to_string(FLAGS_local_utm_zone_id),
                   " +ellps=WGS84 +datum=WGS84 +units=m +no_defs");

  /* function car to obu car status timer callback
   */
  void OnV2xCarStatusTimer();
  void MsgDispatcher();
  bool MsgHandle(int fd);
  std::shared_ptr<::apollo::hdmap::HDMap> hdmap_;
  apollo::canbus::Chassis chassis_msg_;
  apollo::localization::LocalizationEstimate localization_msg_;
  ADCTrajectory planning_msg_;
  int sock_fd_ = 0;
  struct sockaddr_in server_addr_;
  std::atomic<bool> exit_;
  apollo::common::VehicleParam vehicle_param_;
  std::shared_ptr<UDPListener<HikProxy>> listener_ =
      std::make_shared<UDPListener<HikProxy>>();
};

}  // namespace v2x
}  // namespace apollo
