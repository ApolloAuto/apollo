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

#ifndef ONBOARD_DRIVERS_SURESTAR_INCLUDE_SURESTAR_PARSER_surestar_status_H
#define ONBOARD_DRIVERS_SURESTAR_INCLUDE_SURESTAR_PARSER_surestar_status_H

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "cyber/cyber.h"
#include "modules/drivers/lidar_surestar/proto/sensor_surestar.pb.h"
#include "modules/drivers/lidar_surestar/proto/sensor_surestar_conf.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"

namespace apollo {
namespace drivers {
namespace surestar {

constexpr double RADIANS_TO_DEGREES = 180.0 / 3.1415926535897;
constexpr double DEGRESS_TO_RADIANS = 3.1415926535897 / 180.0;

#pragma pack(push, 1)  // Turn off struct padding.

const uint8_t STATUS_WARNING = 87;
const uint8_t STATUS_SPEED_LOW = 254;
const uint8_t STATUS_SPEED_HIGH = 255;
// 600*95%
const uint32_t SPEED_TOL = 570;
const uint32_t STATUS_TYPE_INDEX = 1204;
const uint32_t STATUS_VALUE_INDEX = 1205;

// 64e s3 warning
struct WarningBits {
  uint8_t lens_ontamination : 1;  // 1: need clean, 0: not
  uint8_t unit_hot : 1;           // 1:>58C 0: not
  uint8_t unit_cold : 1;          // 1:<5C 0: not
  uint8_t reserved : 2;
  uint8_t pps_signal : 1;  // 1: present, 0: not
  uint8_t gps_signal : 1;  // 1: present, 0: not
  uint8_t not_used : 1;    // 1: present, 0: not
};

// 64e S3 speed
union MotorSpeed {
  struct {
    uint8_t speed_low;
    uint8_t speed_high;
  };
  uint16_t speed;
};

#pragma pack(pop)  // Back to whatever the previous packing mode was.

// get surestar status from packet by surestar Manual
class SurestarStatus {
 public:
  SurestarStatus();
  ~SurestarStatus() {}
  void get_status(
      const std::shared_ptr<apollo::drivers::Surestar::SurestarScan const>&
          scan);

 private:
  void check_warningbit();
  void check_motor_speed();

  std::string _topic_packets;
  std::string _model;

  std::vector<std::pair<uint8_t, uint8_t>> _status;

  // queue size for ros node pub
  int _queue_size;

  WarningBits* _warning_bits;
  MotorSpeed _motor_speed;
};

}  // namespace surestar
}  // namespace drivers
}  // namespace apollo

#endif
