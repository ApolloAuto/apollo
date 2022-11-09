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

#include "modules/drivers/lidar/robosense/parser/robosense_status.h"

#include <memory>
#include <utility>

namespace apollo {
namespace drivers {
namespace robosense {

/** @brief Constructor. */
RobosenseStatus::RobosenseStatus() {}

void RobosenseStatus::get_status(
    const std::shared_ptr<apollo::drivers::suteng::SutengScan const> &scan) {
  status_.clear();
  // temp hard code for byte position
  for (auto &packet : scan->firing_pkts()) {
    uint8_t status_type = *(reinterpret_cast<uint8_t *>(
        const_cast<char *>(packet.data().data() + STATUS_TYPE_INDEX)));
    uint8_t status_value = *(reinterpret_cast<uint8_t *>(
        const_cast<char *>(packet.data().data() + STATUS_VALUE_INDEX)));
    status_.emplace_back(std::make_pair(status_type, status_value));
  }

  check_warningbit();
  check_motor_speed();
}

void RobosenseStatus::check_warningbit() {
  for (auto &status : status_) {
    if (status.first == STATUS_WARNING) {
      warning_bits_ = reinterpret_cast<WarningBits *>(&status.second);
      ADEBUG << "gps:" << warning_bits_->gps_signal
             << "clean:" << warning_bits_->lens_ontamination
             << "hot:" << warning_bits_->unit_hot
             << "cold:" << warning_bits_->unit_cold;
      if (warning_bits_->gps_signal == 0) {
        AINFO << "suteng no gps signal";
      }
      if (warning_bits_->lens_ontamination == 1) {
        AINFO << "suteng need clean";
      }
      if (warning_bits_->unit_hot == 1) {
        AINFO << "suteng unit is too hot:>58";
      }
      if (warning_bits_->unit_cold == 1) {
        AINFO << "suteng unit is too cold:<5";
      }
      break;
    }
  }
}

void RobosenseStatus::check_motor_speed() {
  int length = status_.size();
  for (int i = 0; i < length - 1; ++i) {
    if (status_[i].first == STATUS_SPEED_LOW &&
        status_[i + 1].first == STATUS_SPEED_HIGH) {
      motor_speed_.speed_low = status_[i].second;
      motor_speed_.speed_high = status_[i + 1].second;
      ADEBUG << "speed:" << motor_speed_.speed;
      if (motor_speed_.speed < SPEED_TOL) {
        AINFO << "suteng unit speed is slow:" << motor_speed_.speed << "<"
              << SPEED_TOL;
      }
      break;
    }
  }
}

}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
