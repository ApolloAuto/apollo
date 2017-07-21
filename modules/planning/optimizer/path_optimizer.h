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
 * @file path_optimizer.h
 **/

#ifndef MODULES_PLANNING_OPTIMIZER_PATH_OPTIMIZER_H_
#define MODULES_PLANNING_OPTIMIZER_PATH_OPTIMIZER_H_

#include "modules/planning/optimizer/optimizer.h"

#include "modules/common/proto/error_code.pb.h"

#include "modules/planning/common/decision_data.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/reference_line/reference_line.h"

namespace apollo {
namespace planning {

class PathOptimizer : public Optimizer {
 public:
  explicit PathOptimizer(const std::string &name);
  virtual ~PathOptimizer() = default;
  virtual apollo::common::ErrorCode optimize(
      const SpeedData &speed_data, const ReferenceLine &reference_line,
      const ::apollo::planning::TrajectoryPoint &init_point,
      DecisionData *const decision_data, PathData *const path_data) = 0;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_OPTIMIZER_PATH_OPTIMIZER_H_
