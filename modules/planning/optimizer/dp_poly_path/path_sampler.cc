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
#include "modules/common/proto/error_code.pb.h"
#include "modules/planning/math/double.h"
#include "modules/planning/reference_line/reference_line.h"

namespace apollo {
namespace planning {

PathSampler::PathSampler(const DpPolyPathConfig &config) : _config(config) {}

::apollo::common::ErrorCode PathSampler::sample(
    const ReferenceLine &reference_line,
    const ::apollo::common::TrajectoryPoint &init_point,
    const common::SLPoint &init_sl_point,
    std::vector<std::vector<common::SLPoint>> *const points) {
  CHECK_NOTNULL(points);
  double step_length = init_point.v();
  step_length = std::min(step_length, _config.step_length_max());
  step_length = std::max(step_length, _config.step_length_min());
  double center_l = init_sl_point.l();
  double accumulated_s = init_sl_point.s();
  for (uint32_t i = 0; i < _config.sample_level(); ++i) {
    std::vector<common::SLPoint> level_points;
    if (center_l < _config.lateral_sample_offset()) {
      center_l = 0.0;
    } else {
      center_l = center_l * _config.lateral_adjust_coeff();
    }
    accumulated_s += step_length;
    double level_start_l =
        center_l -
        _config.lateral_sample_offset() *
            ((_config.sample_points_num_each_level() - 1) >> 1);
    for (uint32_t j = 0; j < _config.sample_points_num_each_level(); ++j) {
      // TODO(haoyang): the lateral value is incorrect

      // TODO(haoyang): no checker no protection
      common::SLPoint sl_point;
      sl_point.set_s(accumulated_s);
      sl_point.set_l(level_start_l + _config.lateral_sample_offset());
      if (reference_line.is_on_road(sl_point)) {
        level_points.push_back(sl_point);
      }
    }
    points->push_back(level_points);
  }
  return ::apollo::common::ErrorCode::PLANNING_OK;
}
}  // namespace planning
}  // namespace apollo
