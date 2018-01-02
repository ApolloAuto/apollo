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

/**
 * @file speed_data.h
 **/
#ifndef MODULES_PLANNING_COMMON_SPEED_SPEED_DATA_H_
#define MODULES_PLANNING_COMMON_SPEED_SPEED_DATA_H_

#include <string>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"

namespace apollo {
namespace planning {

class SpeedData {
 public:
  SpeedData() = default;

  explicit SpeedData(std::vector<common::SpeedPoint> speed_points);

  virtual ~SpeedData() = default;

  const std::vector<common::SpeedPoint>& speed_vector() const;

  void set_speed_vector(std::vector<common::SpeedPoint> speed_points);

  void AppendSpeedPoint(const double s, const double time, const double v,
                        const double a, const double da);

  bool EvaluateByTime(const double time,
                      common::SpeedPoint* const speed_point) const;

  double TotalTime() const;

  bool Empty() const { return speed_vector_.empty(); }

  void Clear();

  virtual std::string DebugString() const;

 private:
  std::vector<common::SpeedPoint> speed_vector_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_SPEED_SPEED_DATA_H_
