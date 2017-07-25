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
#include <vector>

#include "modules/planning/optimizer/dp_poly_path/path_sampler.h"
#include "modules/common/log.h"
#include "modules/planning/math/double.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/common/proto/path_point.pb.h"

namespace apollo {
namespace planning {

using apollo::common::Status;

PathSampler::PathSampler(const DpPolyPathConfig &config) : _config(config) {}

Status PathSampler::sample(
    const ReferenceLine& reference_line,
    const ::apollo::common::TrajectoryPoint& init_point,
    const ::apollo::common::SLPoint& init_sl_point,
    std::vector<std::vector<::apollo::common::SLPoint>>* const points) {
  CHECK_NOTNULL(points);
  double reference_line_length = reference_line.reference_map_line().accumulated_s().back();
  double step_length = init_point.v();
  step_length = std::min(step_length, _config.step_length_max());
  step_length = std::max(step_length, _config.step_length_min());
  double center_l = init_sl_point.l();
  double accumulated_s = init_sl_point.s();
  for (size_t i = 0; i < _config.sample_level(); ++i) {
    std::vector<::apollo::common::SLPoint> level_points;
    if (std::abs(center_l) < _config.lateral_sample_offset()) {
      center_l = 0.0;
    } else {
      center_l = center_l * _config.lateral_adjust_coeff();
    }
    accumulated_s += step_length;
    double level_start_l = center_l
        - _config.lateral_sample_offset() * ((_config.sample_points_num_each_level() - 1) >> 1);
    for (size_t j = 0; j < _config.sample_points_num_each_level(); ++j) {
      ::apollo::common::SLPoint sl_point;
      sl_point.set_s(accumulated_s);
      sl_point.set_l(level_start_l + j * _config.lateral_sample_offset());
      if (reference_line.is_on_road(sl_point)) {
        level_points.push_back(std::move(sl_point));
      }
    }
    if (level_points.empty()) {
      if (accumulated_s > reference_line_length) {
        level_points.emplace_back(reference_line_length, -_config.lateral_sample_offset());
        level_points.emplace_back(reference_line_length, 0.0);
        level_points.emplace_back(reference_line_length, _config.lateral_sample_offset());
        points->push_back(level_points);
        return Status::OK();
      } else {
        level_points.emplace_back(accumulated_s, -_config.lateral_sample_offset());
        level_points.emplace_back(accumulated_s, 0.0);
        level_points.emplace_back(accumulated_s, _config.lateral_sample_offset());
      }
    }
    points->push_back(level_points);
  }
  return Status::OK();
}
}  // namespace planning
}  // namespace apollo
