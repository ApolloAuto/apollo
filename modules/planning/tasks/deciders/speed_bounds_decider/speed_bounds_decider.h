/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include "modules/planning/common/st_graph_data.h"
#include "modules/planning/proto/decider_config.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/speed_bounds_decider_config.pb.h"
#include "modules/planning/tasks/deciders/decider.h"

namespace apollo {
namespace planning {

class SpeedBoundsDecider : public Decider {
 public:
  explicit SpeedBoundsDecider(const TaskConfig& config);

 private:
  apollo::common::Status Process(
      Frame* const frame,
      ReferenceLineInfo* const reference_line_info) override;

  void AddPathEndStop(Frame* const frame,
                      ReferenceLineInfo* const reference_line_info);

  void CheckLaneChangeUrgency(Frame* const frame);

  double SetSpeedFallbackDistance(PathDecision* const path_decision);

  // @brief Rule-based stop for side pass on reverse lane
  void StopOnSidePass(Frame* const frame,
                      ReferenceLineInfo* const reference_line_info);

  // @brief Check if necessary to set stop fence used for nonscenario side pass
  bool CheckSidePassStop(const PathData& path_data,
                         const ReferenceLineInfo& reference_line_info,
                         double* stop_s_on_pathdata);

  // @brief Set stop fence for side pass
  bool BuildSidePassStopFence(const PathData& path_data,
                              const double stop_s_on_pathdata,
                              common::PathPoint* stop_pathpoint,
                              Frame* const frame,
                              ReferenceLineInfo* const reference_line_info);

  bool BuildSidePassStopFence(const common::PathPoint& stop_point,
                              Frame* const frame,
                              ReferenceLineInfo* const reference_line_info);

  // @brief Check if ADV stop at a stop fence
  bool CheckADCStop(const ReferenceLineInfo& reference_line_info,
                    const common::PathPoint& stop_point);

  // @brief Check if needed to check clear again for side pass
  bool CheckClearDone(const ReferenceLineInfo& reference_line_info,
                      const common::PathPoint& stop_point);

  void RecordSTGraphDebug(
      const StGraphData& st_graph_data,
      planning_internal::STGraphDebug* st_graph_debug) const;

 private:
  SpeedBoundsDeciderConfig speed_bounds_config_;
  bool is_clear_to_change_lane_ = false;
};

}  // namespace planning
}  // namespace apollo
