/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
 * @file speed_profile_generator.h
 **/

#pragma once

#include <utility>
#include <vector>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"

namespace apollo {
namespace planning {

class SpeedProfileGenerator {
 public:
  SpeedProfileGenerator() = delete;

  static SpeedData GenerateFallbackSpeed(const EgoInfo* ego_info,
                                         const double stop_distance = 0.0);

  static void FillEnoughSpeedPoints(SpeedData* const speed_data);

  static SpeedData GenerateFixedDistanceCreepProfile(const double distance,
                                                     const double max_speed);

 private:
  static SpeedData GenerateStopProfile(const double init_speed,
                                       const double init_acc);
};

}  // namespace planning
}  // namespace apollo
