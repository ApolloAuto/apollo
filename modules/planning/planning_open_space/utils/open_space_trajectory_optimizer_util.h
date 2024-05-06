/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include <vector>
#include <utility>
#include "cyber/common/log.h"


#include "modules/common/math/box2d.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/vec2d.h"

#include "modules/common/configs/vehicle_config_helper.h"

namespace apollo {
namespace planning {

// park generic

class OpenSpaceTrajectoryOptimizerUtil {
 public:
  /**
   * @brief check if box collision with obstacles
   */
  static bool BoxOverlapObstacles(
    const common::math::Box2d& bounding_box,
    const std::vector<std::vector<common::math::LineSegment2d>>
      obstacles_linesegments_vec);

  /**
   * @brief check if box collision with obstacles
   */
  static void GeneratePointBox(
    const std::vector<std::vector<common::math::LineSegment2d>>
      obstacles_linesegments_vec,
    const std::vector<std::pair<double, double>>& result,
    std::vector<std::vector<common::math::Vec2d>>& box_vec);

  static void PathPointNormalizing(
    double rotate_angle, const common::math::Vec2d& translate_origin,
    double* x, double* y, double* phi);

  static void PathPointDeNormalizing(
    double rotate_angle, const common::math::Vec2d& translate_origin,
    double* x, double* y, double* phi);
};

}  // namespace planning
}  // namespace apollo
