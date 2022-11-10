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

#pragma once

#include <time.h>
#include <unistd.h>

#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include "modules/drivers/lidar/robosense/proto/sensor_suteng.pb.h"
#include "modules/drivers/lidar/robosense/proto/sensor_suteng_conf.pb.h"

#include "modules/drivers/lidar/robosense/driver/driver.h"
#include "modules/drivers/lidar/robosense/lib/data_type.h"
#include "modules/drivers/lidar/robosense/lib/socket_input_16p.h"

namespace apollo {
namespace drivers {
namespace robosense {

class Robosense16PDriver : public RobosenseDriver {
 public:
  explicit Robosense16PDriver(
      const apollo::drivers::suteng::SutengConfig& robo_config);
  ~Robosense16PDriver();

  void init();
  bool poll(const std::shared_ptr<apollo::drivers::suteng::SutengScan>& scan);
  void poll_positioning_packet();
  int poll_msop_sync_count(
      const std::shared_ptr<apollo::drivers::suteng::SutengScan>& scan);

 private:
  std::shared_ptr<SocketInput16P> positioning_input_;
  std::shared_ptr<SocketInput16P> input_16p_;
  std::thread positioning_thread_;
  std::atomic<bool> running_ = {true};
  apollo::drivers::suteng::SutengPacket positioning_pkts_;  // 1hz
};

}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
