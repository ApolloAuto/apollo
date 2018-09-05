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
 * @file qp_spline_path_optimizer.cc
 **/
#include "modules/planning/toolkits/optimizers/qp_spline_path/qp_spline_path_optimizer.h"

#include "modules/common/util/file.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/toolkits/optimizers/qp_spline_path/qp_spline_path_generator.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;

QpSplinePathOptimizer::QpSplinePathOptimizer()
    : PathOptimizer("QpSplinePathOptimizer") {}

bool QpSplinePathOptimizer::Init(const PlanningConfig& config) {
  qp_spline_path_config_ =
      config.lane_follow_scenario_config().qp_spline_path_config();
  std::vector<double> init_knots;
  spline_generator_.reset(
      new Spline1dGenerator(init_knots, qp_spline_path_config_.spline_order()));
  is_init_ = true;
  return true;
}

Status QpSplinePathOptimizer::Process(const SpeedData& speed_data,
                                      const ReferenceLine& reference_line,
                                      const common::TrajectoryPoint& init_point,
                                      PathData* const path_data) {
  if (!is_init_) {
    AERROR << "Please call Init() before Process.";
    return Status(ErrorCode::PLANNING_ERROR, "Not init.");
  }
  QpSplinePathGenerator path_generator(spline_generator_.get(), reference_line,
                                       qp_spline_path_config_,
                                       reference_line_info_->AdcSlBoundary());
  path_generator.SetDebugLogger(reference_line_info_->mutable_debug());
  path_generator.SetChangeLane(reference_line_info_->IsChangeLanePath());

  double boundary_extension = 0.0;
  bool is_final_attempt = false;

  bool ret = path_generator.Generate(
      reference_line_info_->path_decision()->path_obstacles().Items(),
      speed_data, init_point, boundary_extension, is_final_attempt, path_data);
  if (!ret) {
    AERROR << "failed to generate spline path with boundary_extension = 0.";

    boundary_extension = qp_spline_path_config_.cross_lane_lateral_extension();
    is_final_attempt = true;

    ret = path_generator.Generate(
        reference_line_info_->path_decision()->path_obstacles().Items(),
        speed_data, init_point, boundary_extension, is_final_attempt,
        path_data);
    if (!ret) {
      const std::string msg =
          "failed to generate spline path at final attempt.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }

  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
