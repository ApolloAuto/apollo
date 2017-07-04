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
#ifndef MODULES_CONTROL_COMMON_BASE_TYPES_H_
#define MODULES_CONTROL_COMMON_BASE_TYPES_H_

#include <sstream>
#include <string>

/**
 * @namespace apollo::control
 * @brief apollo::control
 */
namespace apollo {
namespace control {

struct PathPoint {
 public:
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double theta = 0.0;
  double kappa = 0.0;
  double s = 0.0;
  std::string DebugString() {
    std::stringstream ss;
    ss << "x:" << x << " y:" << y << " z:" << z << " theta:" << theta
       << " kappa:" << kappa << " s:" << s;
    return ss.str();
  }
};

struct TrajectoryPoint : public PathPoint {
 public:
  double v = 0.0;  // in [m/s]
  double a = 0.0;
  double relative_time = 0.0;
  std::string DebugString() {
    std::stringstream ss;
    ss << PathPoint::DebugString() << " v:" << v << " a:" << a
       << " relative_time:" << relative_time;
    return ss.str();
  }
};

}  // namespace control
}  // namespace apollo

#endif /* MODULES_CONTROL_COMMON_BASE_TYPES_H_ */
