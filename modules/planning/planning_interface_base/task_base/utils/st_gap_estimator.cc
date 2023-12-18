/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
 * @file
 **/

#include "modules/planning/planning_interface_base/task_base/utils/st_gap_estimator.h"

#include <algorithm>
#include <cmath>

#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

// TODO(Jinyun): move to configs
static constexpr double kOvertakeTimeBuffer = 3.0;    // in seconds
static constexpr double kMinOvertakeDistance = 10.0;  // in meters
static constexpr double kDpSafetyDistance = 20.0;     // in meters
static constexpr double kDpSafetyTimeBuffer = 3.0;    // in meters

// TODO(Jinyun): unite gap calculation in dp st and speed decider
double StGapEstimator::EstimateSafeOvertakingGap() { return kDpSafetyDistance; }

double StGapEstimator::EstimateSafeFollowingGap(const double target_obs_speed) {
  return target_obs_speed * kDpSafetyTimeBuffer;
}

double StGapEstimator::EstimateSafeYieldingGap() {
  return FLAGS_yield_distance;
}

// TODO(Jinyun): add more variables to overtaking gap calculation
double StGapEstimator::EstimateProperOvertakingGap(
    const double target_obs_speed, const double adc_speed) {
  const double overtake_distance_s =
      std::fmax(std::fmax(adc_speed, target_obs_speed) * kOvertakeTimeBuffer,
                kMinOvertakeDistance);
  return overtake_distance_s;
}

// TODO(Jinyun): add more variables to follow gap calculation
double StGapEstimator::EstimateProperFollowingGap(const double adc_speed) {
  return std::fmax(adc_speed * FLAGS_follow_time_buffer,
                   FLAGS_follow_min_distance);
}

// TODO(Jinyun): add more variables to yielding gap calculation
double StGapEstimator::EstimateProperYieldingGap() {
  return FLAGS_yield_distance;
}

}  // namespace planning
}  // namespace apollo
