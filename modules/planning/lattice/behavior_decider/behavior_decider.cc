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

#include "modules/planning/lattice/behavior_decider/behavior_decider.h"

#include <string>

#include "gflags/gflags.h"
#include "modules/common/log.h"
#include "modules/common/proto/geometry.pb.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/lattice/behavior_decider/condition_filter.h"
#include "modules/planning/lattice/behavior_decider/scenario_manager.h"
#include "modules/planning/lattice/util/reference_line_matcher.h"

namespace apollo {
namespace planning {

using apollo::common::PathPoint;
using apollo::common::PointENU;

BehaviorDecider::BehaviorDecider(
    std::shared_ptr<PathTimeGraph> ptr_path_time_graph,
    std::shared_ptr<PredictionQuerier> ptr_prediction_obstacles)
    : ptr_path_time_graph_(ptr_path_time_graph),
      ptr_prediction_obstacles_(ptr_prediction_obstacles) {}

PlanningTarget BehaviorDecider::Analyze(
    Frame* frame, ReferenceLineInfo* const reference_line_info,
    const common::TrajectoryPoint& init_planning_point,
    const std::array<double, 3>& lon_init_state,
    const std::vector<common::PathPoint>& discretized_reference_line) {

  CHECK(frame != nullptr);
  CHECK_GT(discretized_reference_line.size(), 0);

  PlanningTarget planning_target;
  if (ScenarioManager::instance()->ComputeWorldDecision(
          frame, reference_line_info, &planning_target) != 0) {
    AERROR << "ComputeWorldDecision error!";
  }

  for (const auto& reference_point : discretized_reference_line) {
    planning_target.mutable_discretized_reference_line()
        ->add_discretized_reference_line_point()
        ->CopyFrom(reference_point);
  }

  CHECK(FLAGS_default_cruise_speed <= FLAGS_planning_upper_speed_limit);

  ConditionFilter condition_filter(lon_init_state,
                                   FLAGS_planning_upper_speed_limit,
                                   ptr_path_time_graph_,
                                   ptr_prediction_obstacles_);

  ComputePathTimeSamplePoints(condition_filter, &planning_target);

  return planning_target;
}

void BehaviorDecider::ComputePathTimeSamplePoints(
    const ConditionFilter& condition_filter,
    PlanningTarget* const planning_target) {
  std::vector<SamplePoint> neighbor_points =
      condition_filter.QueryPathTimeObstacleSamplePoints();
  for (const auto& neighbor_point : neighbor_points) {
    planning_target->add_neighbor_point()->CopyFrom(neighbor_point);
  }

  if (neighbor_points.empty()) {
    ADEBUG << "Sample_bounds empty";
  } else {
    for (const SamplePoint& neighbor_point : neighbor_points) {
      ADEBUG << "Neighbor point: " << neighbor_point.ShortDebugString();
    }
  }
}

void BehaviorDecider::DumpLatticeImage(const int index,
    const common::TrajectoryPoint& init_planning_point,
    const ConditionFilter& condition_filter,
    ReferenceLineInfo* const reference_line_info) {
  apollo::planning_internal::LatticeStTraining st_data;
  double timestamp = init_planning_point.relative_time();
  std::string st_img_name = "DecisionCycle_" +
                            std::to_string(index) + "_" +
                            std::to_string(timestamp);
  if (condition_filter.GenerateLatticeStPixels(&st_data, timestamp,
                                               st_img_name)) {
    ADEBUG << "Created_lattice_st_image_named = " << st_img_name
           << "_for_timestamp = " << timestamp
           << " num_colored_pixels = " << st_data.pixel_size();
    planning_internal::Debug* ptr_debug =
        reference_line_info->mutable_debug();
    ptr_debug->mutable_planning_data()->mutable_lattice_st_image()->CopyFrom(
        st_data);
  }
}

}  // namespace planning
}  // namespace apollo
