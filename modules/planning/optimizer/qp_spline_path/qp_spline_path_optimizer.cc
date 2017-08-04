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

#include "modules/common/util/file.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/optimizer/qp_spline_path/qp_spline_path_generator.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;

QpSplinePathOptimizer::QpSplinePathOptimizer(const std::string& name)
    : PathOptimizer(name) {}

bool QpSplinePathOptimizer::Init() {
  if (!common::util::GetProtoFromFile(FLAGS_qp_spline_path_config_file,
                                      &qp_spline_path_config_)) {
    AERROR << "Failed to load config file for path generator. config file: "
           << FLAGS_qp_spline_path_config_file;
    return false;
  }
  is_init_ = true;
  return true;
}

Status QpSplinePathOptimizer::Process(const SpeedData& speed_data,
                                      const ReferenceLine& reference_line,
                                      const common::TrajectoryPoint& init_point,
                                      PathDecision* const path_decision,
                                      PathData* const path_data) {
  if (!is_init_) {
    AERROR << "Please call Init() before Process.";
    return Status(ErrorCode::PLANNING_ERROR, "Not init.");
  }
  QpSplinePathGenerator path_generator(reference_line, qp_spline_path_config_);

  if (!path_generator.Generate(frame_->GetObstacles().Items(), speed_data,
                               init_point, path_data)) {
    const std::string msg = "failed to generate spline path!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
