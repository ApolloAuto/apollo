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
#include "modules/planning/optimizer/qp_spline_path/qp_spline_path_optimizer.h"

#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;

QpSplinePathOptimizer::QpSplinePathOptimizer(const std::string& name)
    : PathOptimizer(name) {}

bool QpSplinePathOptimizer::Init() {
  if (!_path_generator.Init(FLAGS_qp_spline_path_config_file)) {
    AERROR << "Fail to set config file for path generator.";
    return false;
  }
  is_init_ = true;
  return true;
}

Status QpSplinePathOptimizer::Process(const SpeedData& speed_data,
                                      const ReferenceLine& reference_line,
                                      const common::TrajectoryPoint& init_point,
                                      DecisionData* const decision_data,
                                      PathData* const path_data) {
  if (!is_init_) {
    AERROR << "Please call Init() before Process.";
    return Status(ErrorCode::PLANNING_ERROR, "Not init.");
  }

  if (!_path_generator.generate(reference_line, *decision_data, speed_data,
                                init_point, path_data)) {
    const std::string msg = "failed to generate spline path!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
