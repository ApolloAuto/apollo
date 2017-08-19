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

/**
 * @file
 **/

#ifndef MODULES_PLANNING_COMMON_REFERENCE_LINE_INFO_H_
#define MODULES_PLANNING_COMMON_REFERENCE_LINE_INFO_H_

#include <limits>
#include <list>
#include <string>
#include <vector>

#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/path_decision.h"
#include "modules/planning/common/planning_data.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"

namespace apollo {
namespace planning {

class ReferenceLineInfo {
 public:
  explicit ReferenceLineInfo(const ReferenceLine& reference_line);
  const std::string& Id() const;

  bool AddObstacles(const std::vector<const Obstacle*>& obstacles);
  PathObstacle* AddObstacle(const Obstacle* obstacle);

  // FIXME(all) this interface is temp. solution to make the code work.
  // remove this interface when ready.
  PathDecision* path_decision() { return &path_decision_; }
  const PathDecision& path_decision() const { return path_decision_; }
  const ReferenceLine& reference_line() const { return reference_line_; }

  // TODO remove this inteface when ready.
  void SetTrajectory(const DiscretizedTrajectory& trajectory) {
    discretized_trajectory_ = trajectory;
  }

  PlanningData* mutable_planning_data();
  const PlanningData& planning_data() const;

  const DiscretizedTrajectory& trajectory() const;

  double Cost() const { return cost_; }

  std::unique_ptr<Obstacle> CreateVirtualObstacle(
      const std::string& obstacle_id, const double route_s, const double length,
      const double width, const double height) const;

  bool CombinePathAndSpeedProfile(const double time_resolution,
                                  const double relative_time);

 private:
  std::unique_ptr<PathObstacle> CreatePathObstacle(const Obstacle* obstacle);
  bool InitPerceptionSLBoundary(PathObstacle* path_obstacle);

 private:
  static uint32_t s_reference_line_id_;

  std::string id_;
  /**
   * @brief this is the number that measures the goodness of this reference
   * line.
   * The lower the better.
   * TODO: implement trajectory cost calculation
   */
  double cost_ = std::numeric_limits<double>::infinity();

  const ReferenceLine reference_line_;
  PathDecision path_decision_;
  PlanningData planning_data_;
  DiscretizedTrajectory discretized_trajectory_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_REFERENCE_LINE_INFO_H_
