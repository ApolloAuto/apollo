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

#include "modules/drivers/lidar_surestar/lib/input.h"

#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace surestar {

bool Input::exract_nmea_time_from_packet(const NMEATimePtr& nmea_time,
                                         const uint8_t* bytes) {
  RFANS_HEARTBEAT_S hb;
  memset(&hb, '\0', sizeof(RFANS_HEARTBEAT_S));
  memcpy(&hb, bytes, sizeof(RFANS_HEARTBEAT_S));
  if (hb.pkgflag == 0xe4e3e2e1) {
    nmea_time->year =
        static_cast<uint16_t>((hb.date & 0xFF));  // get year 相对于2000年
    nmea_time->mon = static_cast<uint16_t>((hb.date & 0xFF00) >> 8);  // [1-12]
    nmea_time->day =
        static_cast<uint16_t>((hb.date & 0xFF0000) >> 16);  // [1-31]
    nmea_time->hour =
        static_cast<uint16_t>((hb.date & 0xFF000000) >> 24);          // [0-23]
    nmea_time->min = static_cast<uint16_t>((hb.time & 0xFF));         // [0-59]
    nmea_time->sec = static_cast<uint16_t>((hb.time & 0xFF00) >> 8);  // [0-59]
  }
  // this is only works the state of connecting device.
  if (nmea_time->year < 0 || nmea_time->year > 99 || nmea_time->mon > 12 ||
      nmea_time->mon < 1 || nmea_time->day > 31 || nmea_time->day < 1 ||
      nmea_time->hour > 23 || nmea_time->hour < 0 || nmea_time->min > 59 ||
      nmea_time->min < 0 || nmea_time->sec > 59 || nmea_time->sec < 0) {
    AINFO << "Invalid GPS time:  " << nmea_time->year << "-" << nmea_time->mon
          << "-" << nmea_time->day << " " << nmea_time->hour << ":"
          << nmea_time->min << ":" << nmea_time->sec
          << ", make sure have connected to GPS device";
    return false;
  }
  return true;
}
}  // namespace surestar
}  // namespace drivers
}  // namespace apollo
