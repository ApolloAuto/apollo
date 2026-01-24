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

#include "modules/planning/planning_open_space/utils/open_space_trajectory_optimizer_util.h"

namespace apollo {
namespace planning {

using common::math::Box2d;
using common::math::Vec2d;

bool OpenSpaceTrajectoryOptimizerUtil::BoxOverlapObstacles(
        const Box2d& bounding_box,
        const std::vector<std::vector<common::math::LineSegment2d>> obstacles_linesegments_vec) {
    for (const auto& obstacle_linesegments : obstacles_linesegments_vec) {
        for (const common::math::LineSegment2d& linesegment : obstacle_linesegments) {
            if (bounding_box.HasOverlap(linesegment)) {
                ADEBUG << "collision start at x: " << linesegment.start().x();
                ADEBUG << "collision start at y: " << linesegment.start().y();
                ADEBUG << "collision end at x: " << linesegment.end().x();
                ADEBUG << "collision end at y: " << linesegment.end().y();
                return true;
            }
        }
    }
    return false;
}

void OpenSpaceTrajectoryOptimizerUtil::GeneratePointBox(
        const std::vector<std::vector<common::math::LineSegment2d>> obstacles_linesegments_vec,
        const std::vector<std::pair<double, double>>& result,
        std::vector<std::vector<Vec2d>>& box_vec) {
    auto vehicle_param = common::VehicleConfigHelper::GetConfig().vehicle_param();
    double shift_distance = vehicle_param.length() / 2.0 - vehicle_param.back_edge_to_center();
    double init_length = vehicle_param.length();
    double init_width = vehicle_param.width();
    double rate = 1.0;

    box_vec.clear();
    for (int i = 0; i < result.size() - 1; i++) {
        double x = result[i].first;
        double y = result[i].second;
        double x_next = result[i + 1].first;
        double y_next = result[i + 1].second;
        double heading = std::atan2(y_next - y, x_next - x);
        AINFO << "init pos: " << x << " " << y;
        // ego_box.Shift(shift_vec);
        // x += shift_distance;
        Box2d box({x, y}, heading, init_length, init_width);
        for (int j = 1; j < 10; j++) {
            if (OpenSpaceTrajectoryOptimizerUtil::BoxOverlapObstacles(box, obstacles_linesegments_vec))
                break;
            box = Box2d({x, y}, heading, init_length + j * 0.2, init_width + j * 0.2);
        }
        auto corners = box.GetAllCorners();
        box_vec.push_back(corners);
        // 3-----------2
        //  |         |
        //  |         |
        // 4-----------1
        ADEBUG << "corners 1: " << corners[0].x() << " " << corners[0].y();
        ADEBUG << "corners 2: " << corners[1].x() << " " << corners[1].y();
        ADEBUG << "corners 3: " << corners[2].x() << " " << corners[2].y();
        ADEBUG << "corners 4: " << corners[3].x() << " " << corners[3].y();
    }
}

void OpenSpaceTrajectoryOptimizerUtil::PathPointNormalizing(
        double rotate_angle,
        const Vec2d& translate_origin,
        double* x,
        double* y,
        double* phi) {
    *x -= translate_origin.x();
    *y -= translate_origin.y();
    double tmp_x = *x;
    *x = (*x) * std::cos(-rotate_angle) - (*y) * std::sin(-rotate_angle);
    *y = tmp_x * std::sin(-rotate_angle) + (*y) * std::cos(-rotate_angle);
    *phi = common::math::NormalizeAngle(*phi - rotate_angle);
}

void OpenSpaceTrajectoryOptimizerUtil::PathPointDeNormalizing(
        double rotate_angle,
        const Vec2d& translate_origin,
        double* x,
        double* y,
        double* phi) {
    double tmp_x = *x;
    *x = (*x) * std::cos(rotate_angle) - (*y) * std::sin(rotate_angle);
    *y = tmp_x * std::sin(rotate_angle) + (*y) * std::cos(rotate_angle);
    *x += translate_origin.x();
    *y += translate_origin.y();
    *phi = common::math::NormalizeAngle(*phi + rotate_angle);
}

}  // namespace planning
}  // namespace apollo
