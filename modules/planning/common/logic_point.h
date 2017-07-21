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
 * @file logic_point.h
 **/

#ifndef MODULES_PLANNING_COMMON_LOGIC_POINT_H_
#define MODULES_PLANNING_COMMON_LOGIC_POINT_H_

#include <string>

#include "modules/common/math/vec2d_utils.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

class LogicPoint {
 public:
  explicit LogicPoint(const double x, const double y, const double s,
                      const double heading, const double kappa,
                      const double dkappa);
  double s() const;
  double heading() const;
  double kappa() const;
  double dkappa() const;
  const std::string &lane_id() const;
  void set_lane_id(const std::string &lane_id);

 private:
  apollo::common::Vec2D _point;
  double _s;
  double _heading;
  double _kappa;
  double _dkappa;
  std::string _lane_id;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_LOGIC_POINT_H_
