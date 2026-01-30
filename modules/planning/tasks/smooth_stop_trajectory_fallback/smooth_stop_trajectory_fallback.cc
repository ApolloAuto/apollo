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
 * @file smooth_stop_trajectory_fallback.cc
 **/

#include "modules/planning/tasks/smooth_stop_trajectory_fallback/smooth_stop_trajectory_fallback.h"

#include <algorithm>

#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_base/common/reference_line_info.h"
#include "modules/planning/planning_base/common/speed_profile_generator.h"
#include "modules/planning/tasks/smooth_stop_trajectory_fallback/util/speed_planner.h"

namespace apollo {
namespace planning {

using common::math::Vec2d;

SpeedData SmoothStopTrajectoryFallback::GenerateFallbackSpeed(const EgoInfo* ego_info, const double stop_distance) {
    const double v0 = ego_info->start_point().v();
    const double a0 = ego_info->start_point().a();
    // Use the max value of jerk bound as max jerk value for fallback speed
    // planning for convenience.
    const double max_deceleration = FLAGS_slowdown_profile_deceleration;
    // Get the lower bound to check collision.
    std::vector<std::vector<Vec2d>> lower_bounds;
    GetLowerSTBound(lower_bounds);
    // Check the initial motion state to make sure the vehicle not move backward.
    if (a0 <= 0.0) {
        SpeedData speed_data;
        if (v0 <= 0.0) {
            // Make vehicle stop directly to get status: (v0 = 0, a0 = 0)
            speed_data.AppendSpeedPoint(0.0, 0.0, 0.0, 0.0, 0.0);
            AINFO << "Fallback speed: current speed is already 0! a0: " << a0 << " v0: " << v0;
            SpeedProfileGenerator::FillEnoughSpeedPoints(&speed_data);
            return speed_data;
        } else {
            double square_a0 = a0 * a0;
            double min_jerk_required = square_a0 / (2.0 * v0);
            if (FLAGS_longitudinal_jerk_upper_bound < min_jerk_required) {
                double slow_down_time = -a0 / min_jerk_required;
                // Make vehicle stop directly to get status: (v0 = 0, a0 = 0)
                double s = 0.0;
                double v = 0.0;
                double a = 0.0;
                double square_t = 0.0;
                for (double t = 0.0; t < slow_down_time; t += FLAGS_fallback_time_unit) {
                    square_t = t * t;
                    a = a0 + t * min_jerk_required;
                    v = v0 + a0 * t + 0.5 * min_jerk_required * square_t;
                    s = v0 * t + 0.5 * a0 * square_t + min_jerk_required * square_t * t / 6.0;
                    speed_data.AppendSpeedPoint(s, t, v, a, min_jerk_required);
                }
                if (!IsCollisionWithSpeedBoundaries(speed_data)) {
                    AINFO << "Fallback speed: Make vehicle just stop to zero speed. a0: " << a0 << " v0: " << v0
                          << " new_jerk: " << min_jerk_required;
                    SpeedProfileGenerator::FillEnoughSpeedPoints(&speed_data);
                    return speed_data;
                }
                // Fast stop because of collision.
                SpeedData fast_stop_speed_data;
                SpeedPlanner::FastStop(v0, a0, max_deceleration, &fast_stop_speed_data);
                SpeedProfileGenerator::FillEnoughSpeedPoints(&fast_stop_speed_data);
                AINFO << "Fallback speed: Fast stop because of collision. a0: " << a0 << " v0: " << v0
                      << " dec: " << max_deceleration;
                return fast_stop_speed_data;
            }
        }
    }
    // 1. Stop with max deceleration and check collision.
    SpeedData fast_stop_speed_data;
    SpeedPlanner::FastStop(v0, a0, max_deceleration, &fast_stop_speed_data);
    if (IsCollisionWithSpeedBoundaries(fast_stop_speed_data)) {
        SpeedProfileGenerator::FillEnoughSpeedPoints(&fast_stop_speed_data);
        AINFO << "Fallback speed: Fast stop the vehicle. a0: " << a0 << " v0: " << v0 << " dec: " << max_deceleration;
        return fast_stop_speed_data;
    }

    // 2. Stop smoothly and check collision.
    SpeedData smooth_stop_speed_data;
    SpeedPlanner::SmoothStop(
            v0,
            a0,
            max_deceleration,
            FLAGS_longitudinal_jerk_upper_bound,
            FLAGS_longitudinal_jerk_lower_bound,
            &smooth_stop_speed_data);
    if (IsCollisionWithSpeedBoundaries(smooth_stop_speed_data)) {
        // Need stop more quickly. Try to enlarge max jerk.
        double max_jerk = fabs(max_deceleration / FLAGS_fallback_time_unit);
        smooth_stop_speed_data.clear();
        SpeedPlanner::SmoothStop(v0, a0, max_deceleration, max_jerk, max_jerk, &smooth_stop_speed_data);
        if (IsCollisionWithSpeedBoundaries(smooth_stop_speed_data)) {
            AINFO << "Fallback speed: Hard to stop smoothly and use still fast stop. "
                     "a0: "
                  << a0 << " v0: " << v0 << " dec: " << max_deceleration;
            SpeedProfileGenerator::FillEnoughSpeedPoints(&fast_stop_speed_data);
            return fast_stop_speed_data;
        }
        SpeedData last_no_collision_speed_data = smooth_stop_speed_data;
        // Find the proper max_jerk with binary search.
        double lower_jerk
                = std::min(fabs(FLAGS_longitudinal_jerk_upper_bound), fabs(FLAGS_longitudinal_jerk_lower_bound));
        double upper_jerk = max_jerk;
        double mid_jerk = 0.0;
        const double jerk_error = 1.0;
        while (mid_jerk - lower_jerk > jerk_error) {
            mid_jerk = (lower_jerk + upper_jerk) * 0.5;
            SpeedPlanner::SmoothStop(v0, a0, max_deceleration, mid_jerk, mid_jerk, &smooth_stop_speed_data);
            if (IsCollisionWithSpeedBoundaries(smooth_stop_speed_data)) {
                lower_jerk = mid_jerk;
            } else {
                upper_jerk = mid_jerk;
                last_no_collision_speed_data = smooth_stop_speed_data;
            }
        }
        SpeedProfileGenerator::FillEnoughSpeedPoints(&last_no_collision_speed_data);
        AINFO << "Fallback speed: Find new jerk to stop the vehilce. a0: " << a0 << " v0: " << v0
              << " new jerk: " << mid_jerk;
        return last_no_collision_speed_data;
    }
    AINFO << "Fallback speed: stop smoothly the vehilce. a0: " << a0 << " v0: " << v0 << " dec: " << max_deceleration;
    SpeedProfileGenerator::FillEnoughSpeedPoints(&smooth_stop_speed_data);
    return smooth_stop_speed_data;
}

bool SmoothStopTrajectoryFallback::IsCollisionWithSpeedBoundaries(
        const SpeedData& speed_data) const {
    if (speed_data.empty()) {
        return false;
    }
    const std::vector<const STBoundary*>& st_boundaries = reference_line_info_->st_graph_data().st_boundaries();
    for (const auto& boundary : st_boundaries) {
        for (int i = 0; i < speed_data.size() - 1; i++) {
            Vec2d point1(speed_data[i].s(), speed_data[i].t());
            Vec2d point2(speed_data[i + 1].s(), speed_data[i + 1].t());
            if (boundary->HasOverlap({point1, point2})) {
                return true;
            }
        }
    }
    return false;
}

bool CompareVec2d(const Vec2d& v1, const Vec2d& v2) {
    return (v1.x() < v2.x());
}

void SmoothStopTrajectoryFallback::GetLowerSTBound(std::vector<std::vector<Vec2d>>& lower_bounds) {
    lower_bounds.clear();
    const std::vector<const STBoundary*>& st_boundaries = reference_line_info_->st_graph_data().st_boundaries();
    const double min_time_distance = 0.0001;
    for (const STBoundary* boundary : st_boundaries) {
        std::vector<Vec2d> lower_bound_points;
        const auto& points = boundary->points();
        for (const auto& p : points) {
            bool is_exist = false;
            for (size_t i = 0; i < lower_bound_points.size(); ++i) {
                if (fabs(lower_bound_points[i].x() - p.x()) < min_time_distance) {
                    if (lower_bound_points[i].y() > p.y()) {
                        lower_bound_points[i].set_y(p.y());
                    }
                    is_exist = true;
                    break;
                }
            }
            if (!is_exist) {
                lower_bound_points.emplace_back(p);
            }
        }
        std::sort(lower_bound_points.begin(), lower_bound_points.end(), CompareVec2d);
        lower_bounds.emplace_back(lower_bound_points);
    }
}

}  // namespace planning
}  // namespace apollo
