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

#include "modules/drivers/lidar_surestar/parser/surestar_status.h"

#include <memory>
#include <utility>

namespace apollo {
namespace drivers {
namespace surestar {

/** @brief Constructor. */
SurestarStatus::SurestarStatus() {}

void SurestarStatus::get_status(
    const std::shared_ptr<apollo::drivers::Surestar::SurestarScan const>
        &scan) {
  _status.clear();
  // temp hard code for byte position
  for (auto &packet : scan->firing_pkts()) {
    uint8_t status_type = *(reinterpret_cast<uint8_t *>(
        const_cast<char *>(packet.data().data() + STATUS_TYPE_INDEX)));
    uint8_t status_value = *(reinterpret_cast<uint8_t *>(
        const_cast<char *>(packet.data().data() + STATUS_VALUE_INDEX)));
    _status.emplace_back(std::make_pair(status_type, status_value));
  }

  check_warningbit();
  check_motor_speed();
}

void SurestarStatus::check_warningbit() {
  for (auto &status : _status) {
    if (status.first == STATUS_WARNING) {
      _warning_bits = reinterpret_cast<WarningBits *>(&status.second);
      ADEBUG << "gps:" << _warning_bits->gps_signal
             << "clean:" << _warning_bits->lens_ontamination
             << "hot:" << _warning_bits->unit_hot
             << "cold:" << _warning_bits->unit_cold;
      if (_warning_bits->gps_signal == 0) {
        AINFO << "surestar no gps signal";
      }
      if (_warning_bits->lens_ontamination == 1) {
        AINFO << "surestar need clean";
      }
      if (_warning_bits->unit_hot == 1) {
        AINFO << "surestar unit is too hot:>58";
      }
      if (_warning_bits->unit_cold == 1) {
        AINFO << "surestar unit is too cold:<5";
      }
      break;
    }
  }
}

void SurestarStatus::check_motor_speed() {
  int length = _status.size();
  for (int i = 0; i < length - 1; ++i) {
    if (_status[i].first == STATUS_SPEED_LOW &&
        _status[i + 1].first == STATUS_SPEED_HIGH) {
      _motor_speed.speed_low = _status[i].second;
      _motor_speed.speed_high = _status[i + 1].second;
      ADEBUG << "speed:" << _motor_speed.speed;
      if (_motor_speed.speed < SPEED_TOL) {
        AINFO << "surestar unit speed is slow:" << _motor_speed.speed << "<"
              << SPEED_TOL;
      }
      break;
    }
  }
}

}  // namespace surestar
}  // namespace drivers
}  // namespace apollo
