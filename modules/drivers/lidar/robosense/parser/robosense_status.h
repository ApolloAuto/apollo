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
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"
#include "modules/drivers/lidar/robosense/proto/sensor_suteng.pb.h"
#include "modules/drivers/lidar/robosense/proto/sensor_suteng_conf.pb.h"

#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace robosense {

constexpr double RADIANS_TO_DEGREES = 180.0 / M_PI;
constexpr double DEGRESS_TO_RADIANS = M_PI / 180.0;

const uint8_t STATUS_WARNING = 87;
const uint8_t STATUS_SPEED_LOW = 254;
const uint8_t STATUS_SPEED_HIGH = 255;
// 600*95%
const uint32_t SPEED_TOL = 570;
const uint32_t STATUS_TYPE_INDEX = 1204;
const uint32_t STATUS_VALUE_INDEX = 1205;

// 64e s3 warning
struct alignas(8) WarningBits {
  uint8_t lens_ontamination : 1;  // 1: need clean, 0: not
  uint8_t unit_hot : 1;           // 1:>58C 0: not
  uint8_t unit_cold : 1;          // 1:<5C 0: not
  uint8_t reserved : 2;
  uint8_t pps_signal : 1;  // 1: present, 0: not
  uint8_t gps_signal : 1;  // 1: present, 0: not
  uint8_t not_used : 1;    // 1: present, 0: not
};

// 64e S3 speed
union alignas(8) MotorSpeed {
  struct alignas(8) {
    uint8_t speed_low;
    uint8_t speed_high;
  };
  uint16_t speed;
};

// get suteng status from packet by suteng Manual
class RobosenseStatus {
 public:
  RobosenseStatus();
  ~RobosenseStatus() {}
  void get_status(
      const std::shared_ptr<apollo::drivers::suteng::SutengScan const>& scan);

 private:
  void check_warningbit();
  void check_motor_speed();

  std::string topic_packets_;
  std::string model_;

  std::vector<std::pair<uint8_t, uint8_t>> status_;

  // queue size for ros node pub
  int queue_size_;

  WarningBits* warning_bits_;
  MotorSpeed motor_speed_;
};

}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
