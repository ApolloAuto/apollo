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

#ifndef MODULES_PLANNING_PLANNING_NODE_H_
#define MODULES_PLANNING_PLANNING_NODE_H_

#include <vector>

#include "modules/common/vehicle_state/vehicle_state.h"
#include "modules/planning/common/base_types.h"
#include "modules/planning/planning.h"
#include "modules/planning/proto/planning.pb.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class PlanningNode
 * @brief PlanningNode is a class that
 *        implements a ros node for planning module.
 */
class PlanningNode {
 public:
  /**
   * @brief Constructor
   */
  PlanningNode();

  /**
   * @brief Destructor
   */
  virtual ~PlanningNode();

  /**
   * @brief Start the planning node.
   */
  void Run();

  /**
   * @brief Reset the planning node.
   */
  void Reset();

 private:
  void RunOnce();

  ADCTrajectory ToTrajectoryPb(
      const double header_time,
      const std::vector<TrajectoryPoint>& discretized_trajectory);

  Planning planning_;
};

}  // namespace planning
}  // namespace apollo

#endif /* MODULES_PLANNING_PLANNING_NODE_H_ */
