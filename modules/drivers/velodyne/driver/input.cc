/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/velodyne/driver/input.h"

namespace apollo {
namespace drivers {
namespace velodyne {

bool Input::exract_nmea_time_from_packet(NMEATimePtr nmea_time,
                                         const uint8_t* bytes) {
  unsigned int gprmc_index = 206;

  int field_count = 0;
  int time_field_index = 0;
  int validity_field_index = 0;
  int date_field_index = 0;
  while (++gprmc_index < POSITIONING_DATA_PACKET_SIZE - 6 &&
         bytes[gprmc_index] != '*') {
    if (bytes[gprmc_index] == ',') {
      ++field_count;
      if (field_count == 1 && time_field_index == 0) {
        time_field_index = gprmc_index + 1;
      } else if (field_count == 2 && validity_field_index == 0) {
        validity_field_index = gprmc_index + 1;
        if (bytes[validity_field_index] == 'V') {
          AERROR << "NAV receiver warning, GPS info is invalid!";
          return false;
        }
      } else if (field_count == 9 && date_field_index == 0) {
        date_field_index = gprmc_index + 1;
        break;
      }
    }
  }

  if (gprmc_index == POSITIONING_DATA_PACKET_SIZE - 6) {
    return false;
  }

  nmea_time->year =
      static_cast<uint16_t>((bytes[date_field_index + 4] - '0') * 10 +
                            (bytes[date_field_index + 5] - '0'));
  nmea_time->mon =
      static_cast<uint16_t>((bytes[date_field_index + 2] - '0') * 10 +
                            (bytes[date_field_index + 3] - '0'));
  nmea_time->day = static_cast<uint16_t>((bytes[date_field_index] - '0') * 10 +
                                         (bytes[date_field_index + 1] - '0'));
  nmea_time->hour = static_cast<uint16_t>((bytes[time_field_index] - '0') * 10 +
                                          (bytes[time_field_index + 1] - '0'));
  nmea_time->min =
      static_cast<uint16_t>((bytes[time_field_index + 2] - '0') * 10 +
                            (bytes[time_field_index + 3] - '0'));
  nmea_time->sec =
      static_cast<uint16_t>((bytes[time_field_index + 4] - '0') * 10 +
                            (bytes[time_field_index + 5] - '0'));

  if (nmea_time->year < 0 || nmea_time->year > 99 || nmea_time->mon > 12 ||
      nmea_time->mon < 1 || nmea_time->day > 31 || nmea_time->day < 1 ||
      nmea_time->hour > 23 || nmea_time->hour < 0 || nmea_time->min > 59 ||
      nmea_time->min < 0 || nmea_time->sec > 59 || nmea_time->sec < 0) {
    AERROR << "Invalid GPS time:  " << nmea_time->year << "-" << nmea_time->mon
           << "-" << nmea_time->day << " " << nmea_time->hour << ":"
           << nmea_time->min << ":" << nmea_time->sec
           << ", make sure have connected to GPS device";
    return false;
  }
  return true;
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
