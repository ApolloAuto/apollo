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
 * @file util.cc
 **/

#include <limits>

#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/tasks/traffic_decider/util.h"

namespace apollo {
namespace planning {
namespace util {

double GetADCStopDeceleration(ReferenceLineInfo* const reference_line_info,
                              const double stop_line_s,
                              const double min_pass_s_distance) {
  double adc_speed =
      common::VehicleStateProvider::instance()->linear_velocity();
  if (adc_speed < FLAGS_max_stop_speed) {
    return 0.0;
  }

  double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
  double stop_distance = 0;

  if (stop_line_s > adc_front_edge_s) {
    stop_distance = stop_line_s - adc_front_edge_s;
  } else {
    stop_distance = stop_line_s + min_pass_s_distance - adc_front_edge_s;
  }
  if (stop_distance < 1e-5) {
    return std::numeric_limits<double>::max();
  }
  return (adc_speed * adc_speed) / (2 * stop_distance);
}

}  // namespace util
}  // namespace planning
}  // namespace apollo
