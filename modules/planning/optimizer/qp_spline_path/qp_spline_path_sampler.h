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
 * @file qp_spline_path_sampler.h
 **/

#ifndef BAIDU_IDG_HOUSTON_OPTIMIZER_QP_SPLINE_PATH_OPTIMIZER_QP_SPLINE_PATH_SAMPLER_H_
#define BAIDU_IDG_HOUSTON_OPTIMIZER_QP_SPLINE_PATH_OPTIMIZER_QP_SPLINE_PATH_SAMPLER_H_

#include "common/environment.h"
#include "common/path/frenet_frame_point.h"
#include "common/planning_error.h"
#include "reference_line/reference_line.h"

namespace apollo {
namespace planning {

class QPSplinePathSampler {
 public:
  virtual ErrorCode sample(const Environment& environment,
                           const FrenetFramePoint& init_point,
                           const ReferenceLine& reference_line,
                           const std::size_t num_of_sampling_point,
                           const double s_lower_bound,
                           const double s_upper_bound,
                           std::vector<double>* const sampling_point) = 0;
};
}  // namespace planning
}  // namespace apollo

#endif  // BAIDU_IDG_HOUSTON_OPTIMIZER_QP_SPLINE_PATH_OPTIMIZER_QP_SPLINE_PATH_SAMPLER_H_
