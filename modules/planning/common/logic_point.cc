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
* @file logic_point.cpp
* @brief logic point
*/

#include "modules/planning/common/logic_point.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

LogicPoint::LogicPoint(const double x, const double y, const double s,
                       const double heading, const double kappa,
                       const double dkappa)
    : _point(::apollo::common::math::Vec2DCtor(x, y)),
      _s(s),
      _heading(heading),
      _kappa(kappa),
      _dkappa(dkappa),
      _lane_id("") {}

double LogicPoint::s() const { return _s; }

double LogicPoint::heading() const { return _heading; }

double LogicPoint::kappa() const { return _kappa; }

double LogicPoint::dkappa() const { return _dkappa; }

void LogicPoint::set_lane_id(const std::string &lane_id) { _lane_id = lane_id; }

const std::string &LogicPoint::lane_id() const { return _lane_id; }

}  // namespace planning
}  // namespace apollo
