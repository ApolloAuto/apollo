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

#ifndef MODULES_PLANNING_TASKS_QP_SPLINE_ST_QP_SPLINE_ST_SPEED_OPTIMIZER_H_
#define MODULES_PLANNING_TASKS_QP_SPLINE_ST_QP_SPLINE_ST_SPEED_OPTIMIZER_H_

#include <memory>
#include <string>
#include <vector>

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/qp_st_speed_config.pb.h"
#include "modules/planning/proto/st_boundary_config.pb.h"

#include "modules/planning/math/smoothing_spline/spline_1d_generator.h"
#include "modules/planning/tasks/speed_optimizer.h"
#include "modules/planning/tasks/st_graph/speed_limit_decider.h"
#include "modules/planning/tasks/st_graph/st_boundary_mapper.h"

namespace apollo {
namespace planning {

class QpSplineStSpeedOptimizer : public SpeedOptimizer {
 public:
  QpSplineStSpeedOptimizer();

  bool Init(const PlanningConfig& config) override;

 private:
  common::Status Process(const SLBoundary& adc_sl_boundary,
                         const PathData& path_data,
                         const apollo::common::TrajectoryPoint& init_point,
                         const ReferenceLine& reference_line,
                         const SpeedData& reference_speed_data,
                         PathDecision* const path_decision,
                         SpeedData* const speed_data) override;

  QpStSpeedConfig qp_st_speed_config_;
  StBoundaryConfig st_boundary_config_;
  std::unique_ptr<Spline1dGenerator> spline_generator_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_QP_SPLINE_ST_QP_SPLINE_ST_SPEED_OPTIMIZER_H_
