/******************************************************************************
 * copyright 2020 The Apollo Authors. All Rights Reserved.
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

#ifndef ONBOARD_DRIVERS_SURESTAR_INCLUDE_SURESTAR_DRIVER_DRIVER_H
#define ONBOARD_DRIVERS_SURESTAR_INCLUDE_SURESTAR_DRIVER_DRIVER_H

#include <memory>
#include <string>

#include "modules/drivers/lidar_surestar/lib/data_type.h"
#include "modules/drivers/lidar_surestar/lib/pcap_input.h"
#include "modules/drivers/lidar_surestar/lib/socket_input.h"
#include "modules/drivers/lidar_surestar/proto/sensor_surestar.pb.h"
#include "modules/drivers/lidar_surestar/proto/sensor_surestar_conf.pb.h"

namespace apollo {
namespace drivers {
namespace surestar {

class SurestarDriver {
 public:
  SurestarDriver();
  virtual ~SurestarDriver() {}

  virtual bool poll(
      const std::shared_ptr<apollo::drivers::Surestar::SurestarScan>& scan) {
    return true;
  }
  virtual void init() = 0;

 protected:
  apollo::drivers::surestar::SurestarConfig _config;
  std::shared_ptr<Input> _input;

  bool flags = false;

  uint64_t _basetime;
  uint32_t _last_gps_time;
  uint64_t _last_count;

  int poll_standard(
      const std::shared_ptr<apollo::drivers::Surestar::SurestarScan>& scan);
  int poll_sync_count(
      const std::shared_ptr<apollo::drivers::Surestar::SurestarScan>& scan,
      bool main_frame);
  bool set_base_time();
  void set_base_time_from_nmea_time(const NMEATimePtr& nmea_time,
                                    uint64_t* basetime,
                                    bool use_gps_time = false);
  void update_gps_top_hour(unsigned int current_time);
};

class Surestar16Driver : public SurestarDriver {
 public:
  explicit Surestar16Driver(
      const apollo::drivers::surestar::SurestarConfig& surestar_config);
  ~Surestar16Driver();

  void init();
  bool poll(
      const std::shared_ptr<apollo::drivers::Surestar::SurestarScan>& scan);
  void poll_positioning_packet();

 private:
  std::shared_ptr<Input> _positioning_input;
  std::thread positioning_thread_;
  std::atomic<bool> running_ = {true};
};

class SurestarDriverFactory {
 public:
  static SurestarDriver* create_driver(
      const apollo::drivers::surestar::SurestarConfig& surestar_config);
};
}  // namespace surestar
}  // namespace drivers
}  // namespace apollo

#endif  // ONBOARD_DRIVERS_SURESTAR_INCLUDE_SURESTAR_DRIVER_DRIVER_H
