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
 * @file qp_path_optimizer.h
 **/

#ifndef MODULES_PLANNING_TASKS_QP_SPLINE_PATH_QP_SPLINE_PATH_OPTIMIZER_H_
#define MODULES_PLANNING_TASKS_QP_SPLINE_PATH_QP_SPLINE_PATH_OPTIMIZER_H_

#include <memory>
#include <string>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/qp_spline_path_config.pb.h"

#include "modules/planning/math/smoothing_spline/spline_1d_generator.h"
#include "modules/planning/tasks/path_optimizer.h"

namespace apollo {
namespace planning {

class QpSplinePathOptimizer : public PathOptimizer {
 public:
  QpSplinePathOptimizer();
  bool Init(const PlanningConfig& config) override;

 private:
  apollo::common::Status Process(const SpeedData& speed_data,
                                 const ReferenceLine& reference_line,
                                 const common::TrajectoryPoint& init_point,
                                 PathData* const path_data) override;

 private:
  QpSplinePathConfig qp_spline_path_config_;
  std::unique_ptr<Spline1dGenerator> spline_generator_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_QP_SPLINE_PATH_QP_PATH_OPTIMIZER_H_
