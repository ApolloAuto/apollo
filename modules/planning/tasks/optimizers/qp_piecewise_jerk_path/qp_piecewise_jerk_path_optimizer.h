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
 * @file
 **/

#pragma once

#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/math/finite_element_qp/fem_1d_expanded_jerk_qp_problem.h"
#include "modules/planning/math/finite_element_qp/fem_1d_qp_problem.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/qp_piecewise_jerk_path_config.pb.h"
#include "modules/planning/tasks/optimizers/path_optimizer.h"

namespace apollo {
namespace planning {

class QpPiecewiseJerkPathOptimizer : public PathOptimizer {
 public:
  explicit QpPiecewiseJerkPathOptimizer(const TaskConfig& config);

 private:
  apollo::common::Status Process(const SpeedData& speed_data,
                                 const ReferenceLine& reference_line,
                                 const common::TrajectoryPoint& init_point,
                                 PathData* const path_data) override;

  std::vector<std::tuple<double, double, double>> GetLateralBounds(
      const SLBoundary& adc_sl, const common::FrenetFramePoint& frenet_point,
      const double qp_delta_s, double path_length,
      const ReferenceLine& reference_line,
      const std::vector<const Obstacle*>& obstacles);

  std::vector<std::tuple<double, double, double>>
  GetLateralSecondOrderDerivativeBounds(
      const common::TrajectoryPoint& init_point, const double qp_delta_s);

 private:
  std::unique_ptr<Fem1dQpProblem> fem_1d_qp_;
};

}  // namespace planning
}  // namespace apollo
