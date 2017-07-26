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

#include <vector>

#include "modules/planning/proto/planning.pb.h"

namespace apollo {
namespace planning {

class SpeedData {
 public:
  SpeedData() = default;

  SpeedData(std::vector<SpeedPoint> speed_points);

  const std::vector<SpeedPoint>& speed_vector() const;

  void set_speed_vector(const std::vector<SpeedPoint>& speed_points);

  virtual std::string DebugString() const;

  void add_speed_point(const double s, const double time, const double v,
                       const double a, const double da);

  bool get_speed_point_with_time(const double time,
                                 SpeedPoint* const speed_point) const;

  double total_time() const;

 private:
  SpeedPoint interpolate(const SpeedPoint& left, const SpeedPoint& right,
                         const double weight) const;
  std::uint32_t find_index(const double s) const;

  std::vector<SpeedPoint> speed_vector_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_SPEED_SPEED_DATA_H_
