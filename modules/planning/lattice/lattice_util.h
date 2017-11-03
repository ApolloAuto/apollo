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

// NOTE: This file is created temporarily until lattice merge to master branch

#ifndef MODULES_PLANNING_LATTICE_LATTICE_UTIL_H_
#define MODULES_PLANNING_LATTICE_LATTICE_UTIL_H_

#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/reference_line/reference_point.h"

namespace apollo {
namespace planning {

std::vector<common::PathPoint> ToDiscretizedReferenceLine(
    const std::vector<ReferencePoint>& ref_points);

void ComputeInitFrenetState(
    const common::PathPoint& matched_point,
    const common::TrajectoryPoint& cartesian_state,
    std::array<double, 3>* ptr_s,
    std::array<double, 3>* ptr_d);

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_LATTICE_LATTICE_UTIL_H_
