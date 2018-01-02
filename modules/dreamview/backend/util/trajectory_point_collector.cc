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

#include "modules/dreamview/backend/util/trajectory_point_collector.h"

#include <vector>

#include "modules/common/math/box2d.h"
#include "modules/common/math/vec2d.h"

using apollo::common::VehicleConfigHelper;
using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;
using apollo::common::TrajectoryPoint;

namespace apollo {
namespace dreamview {
namespace util {

void TrajectoryPointCollector::Collect(const TrajectoryPoint &point) {
  if (has_previous_) {
    Object *trajectory = world_->add_planning_trajectory();
    trajectory->set_position_x(previous_.path_point().x());
    trajectory->set_position_y(previous_.path_point().y());
    trajectory->set_heading(
        atan2(point.path_point().y() - previous_.path_point().y(),
              point.path_point().x() - previous_.path_point().x()));

    const auto &vehicle_param =
        VehicleConfigHelper::GetConfig().vehicle_param();
    Box2d trajectory_box(
        {previous_.path_point().x(), previous_.path_point().y()},
        trajectory->heading(), vehicle_param.length(), vehicle_param.width());

    std::vector<Vec2d> corners;
    trajectory_box.GetAllCorners(&corners);
    for (const auto &corner : corners) {
      PolygonPoint *polygon_point = trajectory->add_polygon_point();
      polygon_point->set_x(corner.x());
      polygon_point->set_y(corner.y());
    }

    trajectory->set_speed(point.v());
    trajectory->set_speed_acceleration(point.a());
    trajectory->set_timestamp_sec(point.relative_time());
  }
  previous_ = point;
  has_previous_ = true;
}

}  // namespace util
}  // namespace dreamview
}  // namespace apollo
