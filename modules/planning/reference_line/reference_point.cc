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
 * @file reference_point.cc
 **/

#include "modules/planning/reference_line/reference_point.h"

namespace apollo {
namespace planning {

ReferencePoint::ReferencePoint(const ::apollo::common::math::Vec2d& point,
                               const double heading, const double kappa,
                               const double dkappa, const double lower_bound,
                               const double upper_bound)
    : ::apollo::hdmap::PathPoint(point, heading),
      _kappa(kappa),
      _dkappa(dkappa),
      _lower_bound(lower_bound),
      _upper_bound(upper_bound) {}

ReferencePoint::ReferencePoint(
    const apollo::common::math::Vec2d& point, const double heading,
    const ::apollo::hdmap::LaneWaypoint lane_waypoint)
    : ::apollo::hdmap::PathPoint(point, heading, lane_waypoint) {}

ReferencePoint::ReferencePoint(
    const apollo::common::math::Vec2d& point, const double heading,
    const double kappa, const double dkappa,
    const ::apollo::hdmap::LaneWaypoint lane_waypoint)
    : ::apollo::hdmap::PathPoint(point, heading, lane_waypoint),
      _kappa(kappa),
      _dkappa(dkappa) {}

void ReferencePoint::set_kappa(const double kappa) { _kappa = kappa; }

void ReferencePoint::set_dkappa(const double dkappa) { _dkappa = dkappa; }

void ReferencePoint::set_lower_bound(const double lower_bound) {
  _lower_bound = lower_bound;
}

void ReferencePoint::set_upper_bound(const double upper_bound) {
  _upper_bound = upper_bound;
}

double ReferencePoint::kappa() const { return _kappa; }

double ReferencePoint::dkappa() const { return _dkappa; }

double ReferencePoint::lower_bound() const { return _lower_bound; }

double ReferencePoint::upper_bound() const { return _upper_bound; }

}  // namespace planning
}  // namespace apollo
