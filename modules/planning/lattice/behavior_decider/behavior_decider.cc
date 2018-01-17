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

#ifndef MODULES_PLANNING_LATTICE_BEHAVIOR_DECIDER_BEHAVIOR_DECIDER_H
#define MODULES_PLANNING_LATTICE_BEHAVIOR_DECIDER_BEHAVIOR_DECIDER_H

#include "modules/planning/lattice/behavior_decider/behavior_decider.h"

#include <string>

#include "gflags/gflags.h"
#include "modules/planning/lattice/behavior_decider/scenario_manager.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/lattice/util/lattice_params.h"
#include "modules/planning/lattice/behavior_decider/condition_filter.h"
#include "modules/planning/lattice/util/reference_line_matcher.h"
#include "modules/planning/lattice/util/lattice_util.h"
#include "modules/common/log.h"
#include "modules/common/proto/geometry.pb.h"

namespace apollo {
namespace planning {

using apollo::common::PointENU;
using apollo::common::PathPoint;

BehaviorDecider::BehaviorDecider() {}

void BehaviorDecider::UpdatePathTimeNeighborhood(
    std::shared_ptr<PathTimeNeighborhood> p) {
  path_time_neighborhood_ = p;
}

PlanningTarget BehaviorDecider::Analyze(
    Frame* frame, ReferenceLineInfo* const reference_line_info,
    const common::TrajectoryPoint& init_planning_point,
    const std::array<double, 3>& lon_init_state,
    const std::vector<common::PathPoint>& discretized_reference_line) {
  CHECK(frame != nullptr);
  CHECK_GT(discretized_reference_line.size(), 0);

  PlanningTarget ret;
  if (ScenarioManager::instance()->ComputeWorldDecision(
          frame, reference_line_info, &ret) != 0) {
    AERROR << "ComputeWorldDecision error!";
  }

  for (const auto& reference_point : discretized_reference_line) {
    ret.mutable_discretized_reference_line()
        ->add_discretized_reference_line_point()
        ->CopyFrom(reference_point);
  }

  CHECK(FLAGS_default_cruise_speed <= FLAGS_planning_upper_speed_limit);
  ConditionFilter condition_filter(lon_init_state,
                                   FLAGS_planning_upper_speed_limit,
                                   path_time_neighborhood_);

  std::vector<SampleBound> sample_bounds = condition_filter.QuerySampleBounds();

  static int decision_cycles = 0;
  if (FLAGS_enable_lattice_st_image_dump) {
    apollo::planning_internal::LatticeStTraining st_data;
    double timestamp = init_planning_point.relative_time();
    std::string st_img_name = "DecisionCycle_" +
                              std::to_string(decision_cycles) + "_" +
                              std::to_string(timestamp);
    if (condition_filter.GenerateLatticeStPixels(&st_data, timestamp,
                                                 st_img_name)) {
      AINFO << "  Created_lattice_st_image_named=" << st_img_name
            << "_for_timestamp=" << timestamp
            << " num_colored_pixels=" << st_data.pixel_size();
      planning_internal::Debug* ptr_debug =
          reference_line_info->mutable_debug();
      ptr_debug->mutable_planning_data()->mutable_lattice_st_image()->CopyFrom(
          st_data);
    }
  }
  decision_cycles += 1;

  // Debug SampleBound
  AINFO << "[Printing SampleBound]";
  if (sample_bounds.empty()) {
    AINFO << " ------ sample_bounds empty";
  } else {
    for (const SampleBound& sample_bound : sample_bounds) {
      AINFO << " ------ sample_bound: " << sample_bound.ShortDebugString();
    }
  }

  for (const auto& sample_bound : sample_bounds) {
    ret.add_sample_bound()->CopyFrom(sample_bound);
  }
  return ret;
}

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_DECISION_ANALYZER_H
