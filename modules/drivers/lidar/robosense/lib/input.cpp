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

#include "modules/drivers/lidar/robosense/lib/input.h"

#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace robosense {
// uint64_t Input::last_pkt_stamp = 0;
bool Input::exract_nmea_time_from_packet(const NMEATimePtr& nmea_time,
                                         const uint8_t* bytes) {
  unsigned int field_index = 0;
  nmea_time->year = static_cast<uint16_t>(bytes[field_index]);
  nmea_time->mon = static_cast<uint16_t>(bytes[field_index + 1] & 0x0F);
  nmea_time->day = static_cast<uint16_t>(bytes[field_index + 2] & 0x1F);
  nmea_time->hour = static_cast<uint16_t>(bytes[field_index + 3] & 0x1F);
  nmea_time->min = static_cast<uint16_t>(bytes[field_index + 4] & 0x3F);
  nmea_time->sec = static_cast<uint16_t>(bytes[field_index + 5] & 0x3F);
  nmea_time->msec = static_cast<uint16_t>(
      bytes[field_index + 7] + (256 * (bytes[field_index + 6]) & 0x3F));
  nmea_time->usec = static_cast<uint16_t>(
      bytes[field_index + 9] + (256 * (bytes[field_index + 8]) & 0x3F));

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
}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
