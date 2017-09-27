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
 * @file decision_analyzer.h
 **/

#ifndef MODULES_PLANNING_LATTICE_DECISION_ANALYZER_H
#define MODULES_PLANNING_LATTICE_DECISION_ANALYZER_H

#include "gflags/gflags.h"

#include "modules/planning/common/frame.h"
#include "modules/planning/lattice/planning_target.h"
#include "modules/common/proto/pnc_point.pb.h"

namespace apollo{
namespace planning {

DECLARE_double(default_cruise_speed);

class DecisionAnalyzer {
public:
  DecisionAnalyzer();

  virtual ~DecisionAnalyzer() = default;

  PlanningTarget analyze(Frame* frame,
    const common::TrajectoryPoint& init_planning_point,
    const std::array<double, 3>& lon_init_state,
    const common::PathPoint& matched_reference_line_point,
    const std::vector<common::PathPoint>& reference_line);

private:
  double AdjustCruiseSpeed(double init_s, double init_speed,
    double init_target_speed,
    const std::vector<common::PathPoint>& reference_line) const;
};

} // namespace planning
} // namespace apollo

#endif /* MODULES_PLANNING_DECISION_ANALYZER_H */
