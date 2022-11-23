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
#include <memory>
#include <string>

#include "modules/drivers/lidar/robosense/proto/sensor_suteng.pb.h"
#include "modules/drivers/lidar/robosense/proto/sensor_suteng_conf.pb.h"

#include "modules/drivers/lidar/robosense/lib/data_type.h"
#include "modules/drivers/lidar/robosense/lib/pcap_input.h"
#include "modules/drivers/lidar/robosense/lib/socket_input.h"
#include "modules/drivers/lidar/robosense/lib/socket_input_16p.h"

namespace apollo {
namespace drivers {
namespace robosense {

class RobosenseDriver {
 public:
  RobosenseDriver();
  virtual ~RobosenseDriver() {}

  virtual bool poll(
      const std::shared_ptr<apollo::drivers::suteng::SutengScan>& scan) {
    return true;
  }
  virtual void init() = 0;
  uint64_t start_time() { return start_time_; }

 protected:
  apollo::drivers::suteng::SutengConfig config_;
  std::shared_ptr<Input> input_;

  bool flags = false;

  uint64_t basetime_;
  uint32_t last_gps_time_;
  uint64_t start_time_;
  int poll_standard(
      const std::shared_ptr<apollo::drivers::suteng::SutengScan>& scan);
  int poll_sync_count(
      const std::shared_ptr<apollo::drivers::suteng::SutengScan>& scan,
      bool main_frame);
  uint64_t last_count_;
  bool set_base_time();
  void set_base_time_from_nmea_time(const NMEATimePtr& nmea_time,
                                    uint64_t* basetime,
                                    bool use_gps_time = false);
  void update_gps_top_hour(unsigned int current_time);

  bool cute_angle(apollo::drivers::suteng::SutengPacket* packet);
};

class Robosense16Driver : public RobosenseDriver {
 public:
  explicit Robosense16Driver(
      const apollo::drivers::suteng::SutengConfig& robo_config);
  ~Robosense16Driver();

  void init();
  bool poll(const std::shared_ptr<apollo::drivers::suteng::SutengScan>& scan);
  void poll_positioning_packet();

 private:
  std::shared_ptr<Input> positioning_input_;
  std::thread positioning_thread_;
  std::atomic<bool> running_ = {true};
};

}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
