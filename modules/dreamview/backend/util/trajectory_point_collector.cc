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

#include <cmath>

using apollo::common::TrajectoryPoint;

namespace apollo {
namespace dreamview {
namespace util {

void TrajectoryPointCollector::Collect(const TrajectoryPoint &point,
                                       const double base_time) {
  if (has_previous_) {
    Object *trajectory_point = world_->add_planning_trajectory();
    trajectory_point->set_timestamp_sec(previous_.relative_time() + base_time);
    trajectory_point->set_position_x(previous_.path_point().x());
    trajectory_point->set_position_y(previous_.path_point().y());
    trajectory_point->set_speed(previous_.v());
    trajectory_point->set_speed_acceleration(previous_.a());
    trajectory_point->set_kappa(previous_.path_point().kappa());
    trajectory_point->set_heading(
        std::atan2(point.path_point().y() - previous_.path_point().y(),
                   point.path_point().x() - previous_.path_point().x()));
  }
  previous_ = point;
  has_previous_ = true;
}

}  // namespace util
}  // namespace dreamview
}  // namespace apollo
