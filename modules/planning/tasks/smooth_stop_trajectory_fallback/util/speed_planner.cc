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
 * @file s_shape_speed.cc
 **/

#include "modules/planning/tasks/smooth_stop_trajectory_fallback/util/speed_planner.h"

#include "modules/planning/planning_base/common/speed_profile_generator.h"

namespace apollo {
namespace planning {

void SpeedPlanner::FastStop(double v0, double a0, double max_deceleration, SpeedData* speed_data) {
    CHECK_NOTNULL(speed_data);
    if (v0 <= 0.0 || fabs(max_deceleration) <= 0.0) {
        return;
    }
    max_deceleration = -fabs(max_deceleration);
    double total_time = -v0 / max_deceleration;
    double s = 0.0;
    double v = v0;
    speed_data->AppendSpeedPoint(s, 0.0, v, max_deceleration, 1000.0);
    for (double t = FLAGS_fallback_time_unit; t < total_time; t += FLAGS_fallback_time_unit) {
        v = v0 + max_deceleration * t;
        s = v0 * t + 0.5 * max_deceleration * t * t;
        speed_data->AppendSpeedPoint(s, t, v, max_deceleration, 0.0);
    }
}

void SpeedPlanner::SmoothStop(
        double v0,
        double a0,
        double max_deceleration,
        double max_pos_jerk,
        double max_neg_jerk,
        SpeedData* speed_data) {
    CHECK_NOTNULL(speed_data);
    if (v0 <= 0) {
        return;
    }
    // Normalize the sign of input param.
    max_deceleration = -fabs(max_deceleration);
    max_pos_jerk = fabs(max_pos_jerk);
    max_neg_jerk = -fabs(max_neg_jerk);
    CHECK_GT(0.0, max_deceleration);
    CHECK_GT(max_pos_jerk, 0.0);
    CHECK_GT(0.0, max_neg_jerk);
    double J1 = max_neg_jerk;
    if (a0 < max_deceleration) {
        J1 = max_pos_jerk;
    }
    double J3 = max_pos_jerk;
    double T1 = (max_deceleration - a0) / J1;
    double T3 = -max_deceleration / J3;
    double square_deceleration = max_deceleration * max_deceleration;
    double square_a0 = a0 * a0;
    double dv1 = (square_deceleration - square_a0) / (J1 + J1);
    double dv3 = -square_deceleration / (J3 + J3);
    double dv2 = -v0 - dv1 - dv3;
    if (dv2 < 0.0) {
        // There is the phase when acceleration is constant.
        double T2 = dv2 / max_deceleration;
        double total_time = T1 + T2 + T3;
        double square_T1 = T1 * T1;
        double v1 = v0 + a0 * T1 + 0.5 * J1 * square_T1;
        double s1 = v0 * T1 + 0.5 * a0 * square_T1 + J1 * square_T1 * T1 / 6.0;
        double v2 = v1 + max_deceleration * T2;
        double s2 = s1 + v1 * T2 + 0.5 * max_deceleration * T2 * T2;
        // Add the speed points.
        double cur_time = FLAGS_fallback_time_unit;
        double cur_s = 0.0;
        double cur_v = 0.0;
        double cur_a = 0.0;
        double cur_square_t = 0.0;
        speed_data->AppendSpeedPoint(0.0, 0.0, v0, a0, J1);
        while (cur_time < T1) {
            cur_a = a0 + cur_time * J1;
            cur_square_t = cur_time * cur_time;
            cur_v = v0 + a0 * cur_time + 0.5 * J1 * cur_square_t;
            cur_s = v0 * cur_time + 0.5 * a0 * cur_square_t + J1 * cur_square_t * cur_time / 6.0;
            speed_data->AppendSpeedPoint(cur_s, cur_time, cur_v, cur_a, J1);
            cur_time += FLAGS_fallback_time_unit;
        }
        double delta_time = 0.0;
        double T1_T2 = T1 + T2;
        while (cur_time < T1_T2) {
            delta_time = cur_time - T1;
            cur_v = v1 + max_deceleration * delta_time;
            cur_s = s1 + v1 * delta_time + 0.5 * max_deceleration * delta_time * delta_time;
            speed_data->AppendSpeedPoint(cur_s, cur_time, cur_v, max_deceleration, 0.0);
            cur_time += FLAGS_fallback_time_unit;
        }
        while (cur_time < total_time) {
            delta_time = cur_time - T1_T2;
            cur_square_t = delta_time * delta_time;
            cur_a = max_deceleration + J3 * delta_time;
            cur_v = v2 + delta_time * max_deceleration + 0.5 * J3 * cur_square_t;
            cur_s = s2 + v2 * delta_time + 0.5 * max_deceleration * cur_square_t + J3 * cur_square_t * delta_time / 6.0;
            speed_data->AppendSpeedPoint(cur_s, cur_time, cur_v, cur_a, J3);
            cur_time += FLAGS_fallback_time_unit;
        }
        return;
    }
    // No need to reach max_deceleration.
    square_deceleration = (square_a0 - v0 * 2.0 * J1) / (1.0 - J1 / J3);
    if (square_deceleration < 0.0) {
        max_deceleration = 0.0;
    } else {
        max_deceleration = -sqrt(square_deceleration);
    }
    J1 = max_neg_jerk;
    if (a0 < max_deceleration) {
        J1 = max_pos_jerk;
    }
    T1 = (max_deceleration - a0) / J1;
    T3 = -max_deceleration / J3;
    square_deceleration = max_deceleration * max_deceleration;

    double total_time = T1 + T3;
    double square_T1 = T1 * T1;
    double v1 = v0 + a0 * T1 + 0.5 * J1 * square_T1;
    double s1 = v0 * T1 + 0.5 * a0 * square_T1 + J1 * square_T1 * T1 / 6.0;
    // Add the speed points.
    double cur_time = FLAGS_fallback_time_unit;
    double cur_s = 0.0;
    double cur_v = 0.0;
    double cur_a = 0.0;
    double cur_square_t = 0.0;
    speed_data->AppendSpeedPoint(0.0, 0.0, v0, a0, J1);
    while (cur_time < T1) {
        cur_a = a0 + cur_time * J1;
        cur_square_t = cur_time * cur_time;
        cur_v = v0 + a0 * cur_time + 0.5 * J1 * cur_square_t;
        cur_s = v0 * cur_time + 0.5 * a0 * cur_square_t + J1 * cur_square_t * cur_time / 6.0;
        speed_data->AppendSpeedPoint(cur_s, cur_time, cur_v, cur_a, J1);
        cur_time += FLAGS_fallback_time_unit;
    }
    double delta_time = 0.0;
    while (cur_time < total_time) {
        delta_time = cur_time - T1;
        cur_square_t = delta_time * delta_time;
        cur_a = max_deceleration + J3 * delta_time;
        cur_v = v1 + delta_time * max_deceleration + 0.5 * J3 * cur_square_t;
        cur_s = s1 + v1 * delta_time + 0.5 * max_deceleration * cur_square_t + J3 * cur_square_t * delta_time / 6.0;
        speed_data->AppendSpeedPoint(cur_s, cur_time, cur_v, cur_a, J3);
        cur_time += FLAGS_fallback_time_unit;
    }
}

}  // namespace planning
}  // namespace apollo
