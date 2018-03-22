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

#ifndef MODULES_PLANNING_PLANNER_LATTICE_LATTICE_PLANNER_H_
#define MODULES_PLANNING_PLANNER_LATTICE_LATTICE_PLANNER_H_

#include <vector>

#include "modules/common/status/status.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/lattice/behavior/behavior_decider.h"
#include "modules/planning/math/curve1d/curve1d.h"
#include "modules/planning/planner/planner.h"
#include "modules/planning/proto/planning_config.pb.h"

namespace apollo {
namespace planning {

class LatticePlanner : public Planner {
 public:
  LatticePlanner() = default;

  virtual ~LatticePlanner() = default;

  common::Status Init(const PlanningConfig& config) override {
    return common::Status::OK();
  }

  /**
   * @brief Override function Plan in parent class Planner.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @return OK if planning succeeds; error otherwise.
   */
  common::Status Plan(const common::TrajectoryPoint& planning_init_point,
                      Frame* frame) override;

  /**
   * @brief Override function Plan in parent class Planner.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @param reference_line_info The computed reference line.
   * @return OK if planning succeeds; error otherwise.
   */
  common::Status PlanOnReferenceLine(
      const common::TrajectoryPoint& planning_init_point, Frame* frame,
      ReferenceLineInfo* reference_line_info) override;

 private:
  DiscretizedTrajectory GetFutureTrajectory() const;

  bool MapFutureTrajectoryToSL(
      const DiscretizedTrajectory& future_trajectory,
      const std::vector<apollo::common::PathPoint>& discretized_reference_line,
      std::vector<common::SpeedPoint>* st_points,
      std::vector<common::FrenetFramePoint>* sl_points);
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_PLANNER_LATTICE_LATTICE_PLANNER_H_
