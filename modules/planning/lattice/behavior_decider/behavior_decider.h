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

#ifndef MODULES_PLANNING_LATTICE_BEHAVIOR_DECIDER_BEHAVIOR_DECIDER_H_
#define MODULES_PLANNING_LATTICE_BEHAVIOR_DECIDER_BEHAVIOR_DECIDER_H_

#include <vector>
#include <memory>

#include "modules/planning/common/trajectory/discretized_trajectory.h"
#include "modules/common/math/box2d.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/lattice_structure.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/lattice/behavior_decider/path_time_neighborhood.h"

namespace apollo {
namespace planning {

class BehaviorDecider {
 public:
  BehaviorDecider();

  void UpdatePathTimeNeighborhood(std::shared_ptr<PathTimeNeighborhood> p);

  virtual ~BehaviorDecider() = default;

  PlanningTarget Analyze(
      Frame* frame, ReferenceLineInfo* const reference_line_info,
      const common::TrajectoryPoint& init_planning_point,
      const std::array<double, 3>& lon_init_state,
      const std::vector<common::PathPoint>& discretized_reference_line);

 private:
  std::shared_ptr<PathTimeNeighborhood> path_time_neighborhood_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_LATTICE_BEHAVIOR_DECIDER_BEHAVIOR_DECIDER_H_
