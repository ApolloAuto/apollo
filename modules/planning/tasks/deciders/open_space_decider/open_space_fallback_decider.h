/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/proto/open_space_fallback_decider_config.pb.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Dense"
#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"
#include "modules/planning/tasks/deciders/decider.h"

namespace apollo {
namespace planning {
class OpenSpaceFallbackDecider : public Decider {
 public:
  explicit OpenSpaceFallbackDecider(const TaskConfig& config);

 private:
  apollo::common::Status Process(Frame* frame) override;

  // bool IsCollisionFreeTrajectory(const ADCTrajectory& trajectory_pb);

  void BuildPredictedEnvironment(const std::vector<const Obstacle*>& obstacles,
                                 std::vector<std::vector<common::math::Box2d>>&
                                     predicted_bounding_rectangles);

  bool IsCollisionFreeTrajectory(
      const TrajGearPair& trajectory_pb,
      const std::vector<std::vector<common::math::Box2d>>&
          predicted_bounding_rectangles,
      size_t* current_idx, size_t* first_collision_idx);

  bool QuardraticFormulaLowerSolution(const double a, const double b,
                                      const double c, double* sol);
};

}  // namespace planning
}  // namespace apollo
