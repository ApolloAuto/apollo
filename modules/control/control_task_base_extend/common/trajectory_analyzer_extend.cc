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

#include "modules/control/control_task_base_extend/common/trajectory_analyzer_extend.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "Eigen/Core"

#include "cyber/common/log.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/search.h"
#include "modules/control/control_component/common/control_gflags.h"

using apollo::common::PathPoint;
using apollo::common::TrajectoryPoint;

namespace apollo {
namespace control {

TrajectoryPoint TrajectoryAnalyzerExtend::QueryNearestPointByRelativeStation(
        const TrajectoryPoint &ref_point,
        const double &relative_station) const {
    std::vector<std::pair<const common::TrajectoryPoint, double>> relative_trajectory_point;
    relative_trajectory_point.push_back(std::make_pair(ref_point, relative_station));
    auto func_comp
            = [](const TrajectoryPoint &point,
                 const std::vector<std::pair<const common::TrajectoryPoint, double>> &relative_trajectory_point) {
                  return point.path_point().s()
                          < relative_trajectory_point[0].first.path_point().s() + relative_trajectory_point[0].second;
              };

    auto it_low = std::lower_bound(
            trajectory_points_.begin(), trajectory_points_.end(), relative_trajectory_point, func_comp);

    if (it_low == trajectory_points_.begin()) {
        return trajectory_points_.front();
    }

    if (it_low == trajectory_points_.end()) {
        return trajectory_points_.back();
    }

    if (FLAGS_query_forward_station_point_only) {
        return *it_low;
    } else {
        auto it_lower = it_low - 1;
        if (it_low->path_point().s() - ref_point.path_point().s()
            < ref_point.path_point().s() - it_lower->path_point().s()) {
            return *it_low;
        }
        return *it_lower;
    }
}

}  // namespace control
}  // namespace apollo
