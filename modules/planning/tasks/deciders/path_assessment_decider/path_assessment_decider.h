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

#pragma once

#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/tasks/deciders/decider.h"

namespace apollo {
namespace planning {

class PathAssessmentDecider : public Decider {
 public:
  PathAssessmentDecider(const TaskConfig& config,
                        const std::shared_ptr<DependencyInjector>& injector);

 private:
  /** @brief Every time when Process function is called, it will:
   *   1. Check the validity of regular/fallback paths, and remove if invalid.
   *   2. Analyze the paths and label necessary info for speed planning use.
   *   3. Pick the best one and update it into the reference_line.
   */
  common::Status Process(Frame* const frame,
                         ReferenceLineInfo* const reference_line_info) override;

  /////////////////////////////////////////////////////////////////////////////
  // Below are functions called when executing PathAssessmentDecider.

  bool IsValidRegularPath(const ReferenceLineInfo& reference_line_info,
                          const PathData& path_data);

  bool IsValidFallbackPath(const ReferenceLineInfo& reference_line_info,
                           const PathData& path_data);

  void SetPathInfo(const ReferenceLineInfo& reference_line_info,
                   PathData* const path_data);

  void TrimTailingOutLanePoints(PathData* const path_data);

  /////////////////////////////////////////////////////////////////////////////
  // Below are functions used when checking validity of path.

  bool IsGreatlyOffReferenceLine(const PathData& path_data);

  bool IsGreatlyOffRoad(const ReferenceLineInfo& reference_line_info,
                        const PathData& path_data);

  bool IsCollidingWithStaticObstacles(
      const ReferenceLineInfo& reference_line_info, const PathData& path_data);

  bool IsStopOnReverseNeighborLane(const ReferenceLineInfo& reference_line_info,
                                   const PathData& path_data);

  // * @brief Check if the path ever returns to the self-lane.
  //   * @param reference_line_info
  //   * @param path_data
  //   * @return It returns the last index that the path returns to self-lane.
  //   *   If the path always stays within self-lane, it returns the size()-1.
  //   *   If the path never returns to self-lane, returns -1.
  // int IsReturningToSelfLane(
  // const ReferenceLineInfo& reference_line_info, const PathData& path_data);

  /////////////////////////////////////////////////////////////////////////////
  // Below are functions used for setting path point type info.

  void InitPathPointDecision(
      const PathData& path_data,
      std::vector<std::tuple<double, PathData::PathPointType, double>>* const
          path_point_decision);

  void SetPathPointType(
      const ReferenceLineInfo& reference_line_info, const PathData& path_data,
      const bool is_lane_change_path,
      std::vector<std::tuple<double, PathData::PathPointType, double>>* const
          path_point_decision);

  void SetObstacleDistance(
      const ReferenceLineInfo& reference_line_info, const PathData& path_data,
      std::vector<std::tuple<double, PathData::PathPointType, double>>* const
          path_point_decision);

  void RecordDebugInfo(const PathData& path_data, const std::string& debug_name,
                       ReferenceLineInfo* const reference_line_info);
};

/////////////////////////////////////////////////////////////////////////////
// Below are helper functions.

int ContainsOutOnReverseLane(
    const std::vector<std::tuple<double, PathData::PathPointType, double>>&
        path_point_decision);

int GetBackToInLaneIndex(
    const std::vector<std::tuple<double, PathData::PathPointType, double>>&
        path_point_decision);

bool ComparePathData(const PathData& lhs, const PathData& rhs,
                     const Obstacle* blocking_obstacle);

}  // namespace planning
}  // namespace apollo
