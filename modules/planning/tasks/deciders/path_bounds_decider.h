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

#include <string>
#include <tuple>
#include <vector>

#include "modules/planning/proto/decider_config.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/tasks/deciders/decider.h"

namespace apollo {
namespace planning {

class PathBoundsDecider : public Decider {
 public:
  explicit PathBoundsDecider(const TaskConfig& config);

 private:
  apollo::common::Status Process(
      Frame* frame, ReferenceLineInfo* reference_line_info) override;

  bool InitPathBoundaries(
      const ReferenceLine& reference_line,
      const common::TrajectoryPoint& planning_start_point,
      std::vector<std::tuple<double, double, double>>* const path_boundaries);

  bool GetBoundariesFromRoadsAndADC(
      const ReferenceLine& reference_line,
      const SLBoundary& adc_sl_boundary,
      std::vector<std::tuple<double, double, double>>* const path_boundaries);

  bool GetBoundariesFromStaticObstacles(
      const IndexedList<std::string, Obstacle>& indexed_obstacles,
      std::vector<std::tuple<double, double, double>>* const path_boundaries);

  double GetBufferBetweenADCCenterAndEdge();
};

}  // namespace planning
}  // namespace apollo
