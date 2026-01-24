/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/common/status/status.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/path_boundary.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_bounds_decider_util.h"
#include "modules/planning/planning_interface_base/task_base/task.h"

namespace apollo {
namespace planning {

class PathGeneration : public Task {
 public:
  virtual ~PathGeneration() = default;

  apollo::common::Status Execute(
      Frame* frame, ReferenceLineInfo* reference_line_info) override;

  apollo::common::Status Execute(Frame* frame) override;

 protected:
  virtual apollo::common::Status Process(
      Frame* frame, ReferenceLineInfo* reference_line_info) {
    return apollo::common::Status::OK();
  }

  virtual apollo::common::Status Process(Frame* frame) {
    return apollo::common::Status::OK();
  }

  /**
   * @brief calculate init sl state by planning start point, result will store
   * in init_sl_state_
   */

  void GetStartPointSLState();

  /**
   * @brief add path_boundary debug info for PnC monitor
   */
  void RecordDebugInfo(const PathBound& path_boundaries,
                       const std::string& debug_name,
                       ReferenceLineInfo* const reference_line_info);

  /**
   * @brief add path debug info for PnC monitor
   */
  void RecordDebugInfo(const PathData& path_data, const std::string& debug_name,
                       ReferenceLineInfo* const reference_line_info);
  /**
   * @brief get sl boundary of the point on PathData
   * @param path_data PathData which the point is on
   * @param point_index index of the point on PathData
   * @param sl_boundary output SLBoundary of the point
   *
   * @return True if SLBoundary is found
   */
  bool GetSLBoundary(const PathData& path_data, int point_index,
                     const ReferenceLineInfo* reference_line_info,
                     SLBoundary* const sl_boundary);

  SLState init_sl_state_;
  std::vector<SLPolygon> obs_sl_polygons_;
};

}  // namespace planning
}  // namespace apollo
