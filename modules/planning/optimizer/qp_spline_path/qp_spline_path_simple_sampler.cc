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
 * @file qp_spline_path_simple_sampler.cpp
 **/

#include "optimizer/qp_spline_path_optimizer/qp_spline_path_simple_sampler.h"
#include "common/houston_gflags.h"
#include "common/planning_macros.h"
#include "math/double.h"

namespace apollo {
namespace planning {

ErrorCode QPSplinePathSimpleSampler::sample(
    const Environment& environment, const FrenetFramePoint& init_point,
    const ReferenceLine& reference_line,
    const std::size_t number_of_sampling_point, const double s_lower_bound,
    const double s_upper_bound, std::vector<double>* const sampling_point) {
  // TODO: Haoyang Fan, change to a configurable version
  double sampling_distance =
      std::min(reference_line.reference_map_line().length(), s_upper_bound) -
      init_point.s();
  sampling_distance = std::min(sampling_distance, FLAGS_planning_distance);
  QUIT_IF(Double::compare(sampling_distance, 0.0) <= 0,
          ErrorCode::PLANNING_ERROR_FAILED, Level::ERROR,
          "Failed to allocate init trajectory point SL(%f, %f)\
            on target lane with length %f line is",
          init_point.s(), init_point.l(),
          reference_line.reference_map_line().length());

  QUIT_IF(s_lower_bound > init_point.s(), ErrorCode::PLANNING_ERROR_FAILED,
          Level::ERROR,
          "The coordinate system s lower bound %f is greater than projected "
          "car position %f",
          s_lower_bound, init_point.s());

  double start_s = init_point.s();
  sampling_point->emplace_back(start_s);
  // map sampling point
  QUIT_IF(number_of_sampling_point == 0, ErrorCode::PLANNING_ERROR_FAILED,
          Level::ERROR, "sampling point shall greater than 0");
  const double delta_s = sampling_distance / number_of_sampling_point;

  for (std::size_t i = 1; i <= number_of_sampling_point; ++i) {
    sampling_point->emplace_back(delta_s * i + start_s);
  }
  return ErrorCode::PLANNING_OK;
}
}  // namespace planning
}  // namespace apollo
