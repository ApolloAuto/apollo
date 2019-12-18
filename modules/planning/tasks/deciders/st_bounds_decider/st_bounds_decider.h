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
#include <utility>
#include <vector>

#include "modules/planning/common/frame.h"
#include "modules/planning/common/st_graph_data.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/st_bounds_decider_config.pb.h"
#include "modules/planning/tasks/deciders/decider.h"
#include "modules/planning/tasks/deciders/st_bounds_decider/st_driving_limits.h"
#include "modules/planning/tasks/deciders/st_bounds_decider/st_guide_line.h"
#include "modules/planning/tasks/deciders/st_bounds_decider/st_obstacles_processor.h"

namespace apollo {
namespace planning {

constexpr double kSTBoundsDeciderResolution = 0.1;
constexpr double kSTPassableThreshold = 3.0;

class STBoundsDecider : public Decider {
 public:
  explicit STBoundsDecider(const TaskConfig& config);

 private:
  common::Status Process(Frame* const frame,
                         ReferenceLineInfo* const reference_line_info) override;

  void InitSTBoundsDecider(const Frame& frame,
                           ReferenceLineInfo* const reference_line_info);

  common::Status GenerateFallbackSTBound(
      std::vector<std::tuple<double, double, double>>* const st_bound,
      std::vector<std::tuple<double, double, double>>* const vt_bound);

  common::Status GenerateRegularSTBound(
      std::vector<std::tuple<double, double, double>>* const st_bound,
      std::vector<std::tuple<double, double, double>>* const vt_bound);

  void RemoveInvalidDecisions(
      std::pair<double, double> driving_limit,
      std::vector<
          std::pair<std::tuple<double, double, double>,
                    std::vector<std::pair<std::string, ObjectDecisionType>>>>*
          available_choices);

  void RankDecisions(
      double s_guide_line, std::pair<double, double> driving_limit,
      std::vector<
          std::pair<std::tuple<double, double, double>,
                    std::vector<std::pair<std::string, ObjectDecisionType>>>>*
          available_choices);

  bool BackwardFlatten(
      std::vector<std::tuple<double, double, double>>* const st_bound);

  void RecordSTGraphDebug(
      const std::vector<STBoundary>& st_graph_data,
      const std::vector<std::tuple<double, double, double>>& st_bound,
      planning_internal::STGraphDebug* const st_graph_debug);

 private:
  STBoundsDeciderConfig st_bounds_config_;

  STGuideLine st_guide_line_;
  STDrivingLimits st_driving_limits_;
  STObstaclesProcessor st_obstacles_processor_;
};

}  // namespace planning
}  // namespace apollo
