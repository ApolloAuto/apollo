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

#include <limits>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "modules/planning/planning_base/common/path/path_data.h"
#include "modules/planning/planning_base/common/reference_line_info.h"

namespace apollo {
namespace planning {

// PointDecision contains (s, PathPointType, distance to closest obstacle).
using PathPointDecision = std::tuple<double, PathData::PathPointType, double>;
constexpr double kMinObstacleArea = 1e-4;

class PathAssessmentDeciderUtil {
 public:
  /**
   * @brief Check if the generated path is valid
   * @param reference_line_info is input current reference_line_info
   * @param path_data is input generated path
   */
  static bool IsValidRegularPath(const ReferenceLineInfo& reference_line_info,
                                 const PathData& path_data);

  static bool IsGreatlyOffReferenceLine(const PathData& path_data);

  static bool IsGreatlyOffRoad(const ReferenceLineInfo& reference_line_info,
                               const PathData& path_data);

  static bool IsCollidingWithStaticObstacles(
      const ReferenceLineInfo& reference_line_info, const PathData& path_data);

  static bool IsStopOnReverseNeighborLane(
      const ReferenceLineInfo& reference_line_info, const PathData& path_data);

  /**
   * @brief Init path_point_decision as type
   * @param path_data is input generated path
   * @param type is input init type of point decision
   * @param path_point_decision is output point decision which indicate each
   * point decision in path
   */
  static void InitPathPointDecision(
      const PathData& path_data, const PathData::PathPointType type,
      std::vector<PathPointDecision>* const path_point_decision);

  /**
   * @brief Trim the points at tail of path which away from lane
   * @param path_data is input generated path
   * @param type is input init type of point decision
   * @param path_point_decision is output point decision which indicate each
   * point decision in path
   */
  static void TrimTailingOutLanePoints(PathData* const path_data);
};

}  // namespace planning
}  // namespace apollo
