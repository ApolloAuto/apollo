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

#include "modules/planning/util/util.h"

#include <cmath>
#include <limits>

#include "modules/common/math/linear_interpolation.h"
#include "modules/common/util/util.h"

namespace apollo {
namespace planning {

using apollo::common::VehicleState;
using apollo::routing::RoutingResponse;

bool IsVehicleStateValid(const VehicleState& vehicle_state) {
  if (std::isnan(vehicle_state.x()) || std::isnan(vehicle_state.y()) ||
      std::isnan(vehicle_state.z()) || std::isnan(vehicle_state.heading()) ||
      std::isnan(vehicle_state.kappa()) ||
      std::isnan(vehicle_state.linear_velocity()) ||
      std::isnan(vehicle_state.linear_acceleration())) {
    return false;
  }
  return true;
}

bool IsDifferentRouting(const RoutingResponse& first,
                        const RoutingResponse& second) {
  if (first.has_header() && second.has_header()) {
    if (first.header().sequence_num() != second.header().sequence_num()) {
      return true;
    }
    return false;
  } else {
    return true;
  }
}

bool ComputeSLBoundaryIntersection(const SLBoundary& sl_boundary,
                                   const double s, double* ptr_l_min,
                                   double* ptr_l_max) {
  *ptr_l_min = std::numeric_limits<double>::max();
  *ptr_l_max = -std::numeric_limits<double>::max();

  // invalid polygon
  if (sl_boundary.boundary_point_size() < 3) {
    return false;
  }

  bool has_intersection = false;
  for (auto i = 0; i < sl_boundary.boundary_point_size(); ++i) {
    auto j = (i + 1) % sl_boundary.boundary_point_size();
    const auto& p0 = sl_boundary.boundary_point(i);
    const auto& p1 = sl_boundary.boundary_point(j);

    if (common::util::WithinBound<double>(std::fmin(p0.s(), p1.s()),
                                          std::fmax(p0.s(), p1.s()), s)) {
      has_intersection = true;
      auto l = common::math::lerp<double>(p0.l(), p0.s(), p1.l(), p1.s(), s);
      if (l < *ptr_l_min) {
        *ptr_l_min = l;
      }
      if (l > *ptr_l_max) {
        *ptr_l_max = l;
      }
    }
  }
  return has_intersection;
}

}  // namespace planning
}  // namespace apollo
