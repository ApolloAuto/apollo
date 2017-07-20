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

#ifndef MODULES_PLANNING_PLANNER_RTK_REPLAY_PLANNER_H_
#define MODULES_PLANNING_PLANNER_RTK_REPLAY_PLANNER_H_

#include <string>
#include <vector>

#include "modules/planning/planner/planner.h"

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
class RTKReplayPlanner : public Planner {
 public:
  /**
   * @brief Constructor
   */
  RTKReplayPlanner();

  /**
   * @brief Destructor
   */
  virtual ~RTKReplayPlanner() = default;

  /**
   * @brief Overrode function Plan in parent class Planner.
   * @param start_point The trajectory point where planning starts
   * @param discretized_trajectory The computed trajectory
   * @return true if planning succeeds; false otherwise.
   */
  bool MakePlan(
      const apollo::common::TrajectoryPoint& start_point,
      std::vector<apollo::common::TrajectoryPoint>* ptr_trajectory) override;

  /**
   * @brief Read the recorded trajectory file.
   * @param filename The name of the trajectory file.
   */
  void ReadTrajectoryFile(const std::string &filename);

 private:
  std::size_t QueryPositionMatchedPoint(
      const apollo::common::TrajectoryPoint &start_point,
      const std::vector<apollo::common::TrajectoryPoint> &trajectory) const;

  std::vector<apollo::common::TrajectoryPoint> complete_rtk_trajectory_;
};

}  // namespace planning
}  // namespace apollo

#endif /* MODULES_PLANNING_PLANNER_RTK_REPLAY_PLANNER_H_ */
