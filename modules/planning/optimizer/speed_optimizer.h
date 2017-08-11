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
 * @file speed_optimizer.h
 **/

#ifndef MODULES_PLANNING_OPTIMIZER_SPEED_OPTIMIZER_H_
#define MODULES_PLANNING_OPTIMIZER_SPEED_OPTIMIZER_H_

#include <string>

#include "modules/planning/optimizer/optimizer.h"

#include "modules/common/status/status.h"
#include "modules/planning/optimizer/st_graph/st_graph_boundary.h"

namespace apollo {
namespace planning {

class SpeedLimit;
class SpeedData;

class SpeedOptimizer : public Optimizer {
 public:
  explicit SpeedOptimizer(const std::string& name);
  virtual ~SpeedOptimizer() = default;
  apollo::common::Status Optimize(Frame* frame) override;

 protected:
  virtual apollo::common::Status Process(
      const PathData& path_data, const common::TrajectoryPoint& init_point,
      const ReferenceLine& reference_line, PathDecision* const path_decision,
      SpeedData* const speed_data) = 0;

  void RecordSTGraphDebug(const std::vector<StGraphBoundary>& boundaries,
    const SpeedLimit& speed_limits, const SpeedData& speed_data);

};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_OPTIMIZER_SPEED_OPTIMIZER_H_
