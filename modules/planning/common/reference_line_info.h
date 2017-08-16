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

#include <list>
#include <string>
#include <vector>

#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/path_decision.h"
#include "modules/planning/common/speed/speed_data.h"

namespace apollo {
namespace planning {

class ReferenceLineInfo {
 public:
  explicit ReferenceLineInfo(const ReferenceLine& reference_line);
  bool AddObstacles(const std::vector<const Obstacle*>& obstacles);
  bool AddObstacle(const Obstacle* obstacle);
  // FIXME(all) this interface is temp. solution to make the code work.
  // remove this interface when ready.
  PathDecision* path_decision() { return &path_decision_; }

 private:
  std::unique_ptr<PathObstacle> CreatePathObstacle(const Obstacle* obstacle);
  bool InitPerceptionSLBoundary(PathObstacle* path_obstacle);

  const ReferenceLine reference_line_;
  PathDecision path_decision_;
  PathData path_data_;
  SpeedData speed_dta_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_REFERENCE_LINE_INFO_H_
