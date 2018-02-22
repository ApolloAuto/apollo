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

#ifndef MODULES_PLANNING_LATTICE_BEHAVIOR_DECIDER_CONDITION_FILTER_H_
#define MODULES_PLANNING_LATTICE_BEHAVIOR_DECIDER_CONDITION_FILTER_H_

#include <array>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "modules/planning/lattice/behavior_decider/feasible_region.h"
#include "modules/planning/lattice/behavior_decider/path_time_graph.h"
#include "modules/planning/proto/lattice_structure.pb.h"
#include "modules/planning/proto/planning_internal.pb.h"

namespace apollo {
namespace planning {

class ConditionFilter {
 public:
  ConditionFilter(const std::array<double, 3>& init_s, const double speed_limit,
                  std::shared_ptr<PathTimeGraph> path_time_graph);

  std::vector<SamplePoint> QueryPathTimeObstacleSamplePoints() const;

  bool GenerateLatticeStPixels(
      apollo::planning_internal::LatticeStTraining* st_data, double timestamp,
      std::string st_img_name) const;

  bool WithinObstacleSt(double s, double t) const;

 private:
  std::array<double, 3> init_s_;

  FeasibleRegion feasible_region_;

  std::shared_ptr<PathTimeGraph> ptr_path_time_graph_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_LATTICE_BEHAVIOR_DECIDER_CONDITION_FILTER_H_
