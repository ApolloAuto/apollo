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

#ifndef MODULES_PLANNING_PLANNER_EM_PLANNER_H_
#define MODULES_PLANNING_PLANNER_EM_PLANNER_H_

#include "modules/planning/planner/planner.h"

#include <string>
#include <vector>

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class EMPlanner
 * @brief EMPlanner is an expectation maximization planner.
 */

class EMPlanner : public Planner {
 public:
  /**
   * @brief Constructor
   */
  EMPlanner();

  /**
   * @brief Destructor
   */
  virtual ~EMPlanner() = default;

  /**
   * @brief Overrode function Plan in parent class Planner.
   * @param start_point The trajectory point where planning starts
   * @param discretized_trajectory The computed trajectory
   * @return true if planning succeeds; false otherwise.
   */
  bool Plan(const apollo::common::TrajectoryPoint& start_point,
            std::vector<apollo::common::TrajectoryPoint>* trajectory) override;

 private:
};

}  // namespace planning
}  // nameapace apollo

#endif /* MODULES_PLANNING_PLANNER_EM_PLANNER_H_ */
