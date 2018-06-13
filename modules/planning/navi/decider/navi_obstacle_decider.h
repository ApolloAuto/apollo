/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
 * @brief This file provides the declaration of the class "NaviSpeedDecider".
 */

#ifndef MODULES_PLANNING_NAVI_NAVI_OBSTACLE_DECIDER_H_
#define MODULES_PLANNING_NAVI_NAVI_OBSTACLE_DECIDER_H_

#include <string>

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/status/status.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/tasks/task.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class NaviObstacleDecider
 * @brief NaviObstacleDecider is used to make appropriate decisions for
 * obstacles around the vehicle in navigation mode.
 * Note that NaviObstacleDecider is only used in navigation mode (turn on
 * navigation mode by setting "FLAGS_use_navigation_mode" to "true") and do not
 * use it in standard mode.
 */
class NaviObstacleDecider : public Task {
 public:
  NaviObstacleDecider();
  virtual ~NaviObstacleDecider() = default;
  /**
   * @brief Overrided implementation of the virtual function "Execute" in the
   * base class "Task".
   * @param frame Current planning frame.
   * @param reference_line_info Currently available reference line information.
   * @return Status::OK() if a suitable path is created; error otherwise.
   */
  apollo::common::Status Execute(
      Frame *frame, ReferenceLineInfo *reference_line_info) override;

 private:
  apollo::common::Status Process();
  void RecordDebugInfo(const PathData &path_data);

  // TODO(all): Add your member functions and variables.
};

}  // namespace planning
}  // namespace apollo

#endif /* MODULES_PLANNING_NAVI_NAVI_OBSTACLE_DECIDER_H_ */
