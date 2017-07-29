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
 * @file sampler.cpp
 **/
#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "modules/planning/optimizer/dp_poly_path/path_sampler.h"

#include "modules/common/proto/path_point.pb.h"

#include "modules/common/util/util.h"

namespace apollo {
namespace planning {

using Status = apollo::common::Status;
using SLPoint = apollo::common::SLPoint;

PathSampler::PathSampler(const DpPolyPathConfig& config) : config_(config) {}

bool PathSampler::sample(
    const ReferenceLine& reference_line,
    const ::apollo::common::TrajectoryPoint& init_point,
    std::vector<std::vector<::apollo::common::SLPoint>>* const points) {
  if (!points) {
    AERROR << "The provided points are null";
    return false;
  }
  ::apollo::common::SLPoint init_sl_point;
  common::math::Vec2d init_point_vec2d{init_point.path_point().x(),
                                       init_point.path_point().y()};
  if (!reference_line.get_point_in_frenet_frame(init_point_vec2d,
                                                &init_sl_point)) {
    AERROR << "Failed to get sl point from point "
           << init_point_vec2d.DebugString();
    return false;
  }
  const double reference_line_length =
      reference_line.reference_map_line().accumulated_s().back();
  double level_distance =
      std::fmax(config_.step_length_min(),
                std::fmin(init_point.v(), config_.step_length_max()));

  double accumulated_s = init_sl_point.s();
  for (size_t i = 0;
       i < config_.sample_level() && accumulated_s < reference_line_length;
       ++i) {
    std::vector<SLPoint> level_points;
    accumulated_s += level_distance;
    double s = std::fmin(accumulated_s, reference_line_length);

    int32_t num =
        static_cast<int32_t>(config_.sample_points_num_each_level() / 2);
    for (int32_t j = -num; j < num + 1; ++j) {
      double l = config_.lateral_sample_offset() * j;
      SLPoint sl = common::util::MakeSLPoint(s, l);
      if (reference_line.is_on_road(sl)) {
        level_points.push_back(sl);
      }
    }
    if (!level_points.empty()) {
      points->push_back(level_points);
    }
  }
  return true;
}
}  // namespace planning
}  // namespace apollo
