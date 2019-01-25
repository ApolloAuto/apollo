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
 * @file: poly_vt_speed_optimizer.h
 * @brief: sampling based piecewise polynomial velocity optimizer
 **/

#pragma once

#include "modules/common/status/status.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/tasks/optimizers/st_graph/st_graph_data.h"
#include "modules/planning/tasks/task.h"

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/poly_vt_speed_config.pb.h"
#include "modules/planning/proto/st_boundary_config.pb.h"

namespace apollo {
namespace planning {

class PolyVTSpeedOptimizer : public Task {
 public:
  explicit PolyVTSpeedOptimizer(const TaskConfig& config);
  virtual ~PolyVTSpeedOptimizer() = default;
  apollo::common::Status Execute(
      Frame* frame, ReferenceLineInfo* reference_line_info) override;

  void RecordSTGraphDebug(const StGraphData& st_graph_data,
                          planning_internal::STGraphDebug* stGraphDebug) const;

  void RecordDebugInfo(const SpeedData& speed_data);

 private:
  StBoundaryConfig st_boundary_config_;
};

}  // namespace planning
}  // namespace apollo
