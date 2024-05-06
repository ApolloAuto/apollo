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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/planning/planning_base/proto/planning_config.pb.h"

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/reference_line_info.h"
#include "modules/planning/planning_interface_base/planner_base/planner.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class RTKReplayPlanner
 * @brief RTKReplayPlanner is a derived class of Planner.
 *        It reads a recorded trajectory from a trajectory file and
 *        outputs proper segment of the trajectory according to vehicle
 * position.
 */
class RTKReplayPlanner : public PlannerWithReferenceLine {
 public:
  /**
   * @brief Destructor
   */
  virtual ~RTKReplayPlanner() = default;

  std::string Name() override { return "RTK"; }

  apollo::common::Status Init(
      const std::shared_ptr<DependencyInjector>& injector,
      const std::string& config_path = "") override;

  void Stop() override {}

  /**
   * @brief Override function Plan in parent class Planner.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @return OK if planning succeeds; error otherwise.
   */
  apollo::common::Status Plan(
      const common::TrajectoryPoint& planning_init_point, Frame* frame,
      ADCTrajectory* ptr_computed_trajectory) override;

  /**
   * @brief Override function Plan in parent class Planner.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @param reference_line_info The computed reference line.
   * @return OK if planning succeeds; error otherwise.
   */
  apollo::common::Status PlanOnReferenceLine(
      const common::TrajectoryPoint& planning_init_point, Frame* frame,
      ReferenceLineInfo* reference_line_info) override;
  /**
   * @brief Read the recorded trajectory file.
   * @param filename The name of the trajectory file.
   */
  void ReadTrajectoryFile(const std::string& filename);

 private:
  std::uint32_t QueryPositionMatchedPoint(
      const common::TrajectoryPoint& start_point,
      const std::vector<common::TrajectoryPoint>& trajectory) const;

  std::vector<common::TrajectoryPoint> complete_rtk_trajectory_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::RTKReplayPlanner,
                                     Planner)

}  // namespace planning
}  // namespace apollo
