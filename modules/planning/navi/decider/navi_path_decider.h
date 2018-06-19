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
 * @brief This file provides the declaration of the class "NaviPathDecider".
 */

#ifndef MODULES_PLANNING_NAVI_NAVI_PATH_DECIDER_H_
#define MODULES_PLANNING_NAVI_NAVI_PATH_DECIDER_H_

#include <string>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/status/status.h"
#include "modules/planning/navi/common/local_path.h"
#include "modules/planning/proto/navi_path_decider_config.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/tasks/task.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class NaviPathDecider
 * @brief NaviPathDecider is used to generate the local driving path of the
.* vehicle in navigation mode.
 * Note that NaviPathDecider is only used in navigation mode (turn on navigation
 * mode by setting "FLAGS_use_navigation_mode" to "true") and do not use it in
 * standard mode.
 */
class NaviPathDecider : public Task {
 public:
  NaviPathDecider();
  virtual ~NaviPathDecider() = default;

  bool Init(const PlanningConfig &config) override;

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
  /**
   * @brief generate path information for trajectory plan in navigation mode.
   * @param ref_path_data path data information in the reference line.
   * @param path_decision path decision information provided by perception.
   * @param init_point start planning point.
   * @param path_data output path plan information based on FLU coordinate
   * system
   * @return Status::OK() if a suitable path is created; error otherwise.
   */
  apollo::common::Status Process(const ReferenceLine &reference_line,
                                 const common::TrajectoryPoint &init_point,
                                 PathDecision *const path_decision,
                                 PathData *const path_data);

  /**
   * @brief take a section of the reference line as the initial path trajectory.
   * @param reference_line input reference line.
   * @return LocalPath.
   */
  LocalPath GetLocalPath(const ReferenceLine &reference_line);

  /**
   * @brief get init y of plan path trajectory in FLU coordinate.
   * @param init_y in LocalPath.
   * @return init y in FLU coordinate.
   */
  double SmoothInitY(const double init_y);

  void RecordDebugInfo(const PathData &path_data);

  // TODO(all): Add your member functions and variables.
 private:
  NaviPathDeciderConfig config_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_NAVI_NAVI_PATH_DECIDER_H_
