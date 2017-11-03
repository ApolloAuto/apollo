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

#ifndef MODULES_PLANNING_PLANNER_LATTICE_LATTICE_PLANNER_H_
#define MODULES_PLANNING_PLANNER_LATTICE_LATTICE_PLANNER_H_

#include <vector>

#include "modules/planning/common/frame.h"
#include "modules/planning/lattice/behavior_decider/behavior_decider.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/planner/planner.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/math/curve1d/curve1d.h"

namespace apollo {
namespace planning {

class LatticePlanner : public Planner {
 public:
  LatticePlanner();

  virtual ~LatticePlanner() = default;

  common::Status Init(const PlanningConfig& config) override;

  common::Status Plan(const common::TrajectoryPoint& planning_init_point,
                      Frame* frame,
                      ReferenceLineInfo* reference_line_info) override;

 private:
  DiscretizedTrajectory CombineTrajectory(
      const std::vector<common::PathPoint>& reference_line,
      const Curve1d& lon_trajectory, const Curve1d& lat_trajectory,
      const double init_relative_time = 0.0) const;

  BehaviorDecider decider_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_PLANNER_LATTICE_LATTICE_PLANNER_H_
