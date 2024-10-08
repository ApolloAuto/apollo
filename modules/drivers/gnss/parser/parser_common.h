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

#include <cmath>
#include <limits>

#include "modules/common_msgs/basic_msgs/geometry.pb.h"

#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace gnss {

constexpr int SECONDS_PER_WEEK = 60 * 60 * 24 * 7;

constexpr double DEG_TO_RAD = M_PI / 180.0;

constexpr double EPS = 1e-8;

constexpr float FLOAT_NAN = std::numeric_limits<float>::quiet_NaN();

constexpr double azimuth_deg_to_yaw_rad(double azimuth) {
  return (90.0 - azimuth) * DEG_TO_RAD;
}

// A helper that fills an Point3D object (which uses the FLU frame) using RFU
// measurements.
inline void rfu_to_flu(double r, double f, double u,
                       ::apollo::common::Point3D* flu) {
  flu->set_x(f);
  flu->set_y(-r);
  flu->set_z(u);
}

constexpr uint64_t PERIOD_NS_1HZ = 900 * 1e6;
class RateControl {
 public:
  explicit RateControl(uint64_t period_ns) : period_ns_(period_ns) {}
  bool check() {
    auto now = cyber::Time::Now().ToNanosecond();
    if (now - latest_ns_ > period_ns_) {
      latest_ns_ = now;
      return true;
    }
    return false;
  }

 private:
  uint64_t period_ns_;
  uint64_t latest_ns_ = 0;
};

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
