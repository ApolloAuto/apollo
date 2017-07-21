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

#include "modules/planning/optimizer/optimizer.h"

#include "modules/common/proto/error_code.pb.h"
#include "modules/planning/common/decision_data.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/planning_data.h"
#include "modules/planning/common/speed/speed_data.h"

namespace apollo {
namespace planning {

class SpeedOptimizer : public Optimizer {
 public:
  explicit SpeedOptimizer(const std::string& name);
  virtual ~SpeedOptimizer() = default;
  virtual common::ErrorCode optimize(const PathData& path_data,
                                     const TrajectoryPoint& init_point,
                                     DecisionData* const decision_data,
                                     SpeedData* const speed_data) const = 0;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_OPTIMIZER_SPEED_OPTIMIZER_H_
