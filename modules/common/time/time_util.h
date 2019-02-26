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

#pragma once

#include <sys/time.h>
#include <iomanip>

#include "cyber/common/macros.h"

namespace apollo {
namespace common {
namespace time {

#define GLOG_TIMESTAMP(timestamp) \
  std::fixed << std::setprecision(9) << timestamp

class TimeUtil {
 public:
  // @brief: UNIX timestamp to GPS timestamp, in seconds.
  static double Unix2gps(double unix_time) {
    double gps_time = unix_time - UNIX_GPS_DIFF;
    if (unix_time < LEAP_SECOND_TIMESTAMP) {
      gps_time -= 1.0;
    }
    return gps_time;
  }

  // @brief: GPS timestamp to UNIX timestamp, in seconds.
  static double Gps2unix(double gps_time) {
    double unix_time = gps_time + UNIX_GPS_DIFF;
    if (unix_time + 1 < LEAP_SECOND_TIMESTAMP) {
      unix_time += 1.0;
    }
    return unix_time;
  }

  static double GetCurrentTime() {
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    const double timestamp =
        static_cast<double>(tv.tv_sec * 1000000 + tv.tv_usec);
    return timestamp / 1000000;
  }

 private:
  // unix timestamp(1970.01.01) is different from gps timestamp(1980.01.06)
  static const int UNIX_GPS_DIFF = 315964782;
  // unix timestamp(2016.12.31 23:59:59(60) UTC/GMT)
  static const int LEAP_SECOND_TIMESTAMP = 1483228799;

  DISALLOW_COPY_AND_ASSIGN(TimeUtil);
};

}  // namespace time
}  // namespace common
}  // namespace apollo
