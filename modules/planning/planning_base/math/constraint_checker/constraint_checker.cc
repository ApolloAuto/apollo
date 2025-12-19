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
 * @file constraint_checker.cc
 **/

#include "modules/planning/planning_base/math/constraint_checker/constraint_checker.h"

#include "cyber/common/log.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

namespace {
// 判断值 v 是否在 [lower, upper] 闭区间内
template <typename T>
bool WithinRange(const T v, const T lower, const T upper) {
  return lower <= v && v <= upper;
}
}  // namespace

//检查输入轨迹是否满足所有预设的物理和舒适性约束
ConstraintChecker::Result ConstraintChecker::ValidTrajectory(
    const DiscretizedTrajectory& trajectory) {
  const double kMaxCheckRelativeTime = FLAGS_trajectory_time_length;
  // 逐点检查基本状态量（速度、加速度、曲率）
  for (const auto& p : trajectory) {
    double t = p.relative_time();
    // 只检查轨迹中 相对时间 ≤ FLAGS_trajectory_time_length 的部分（通常为 8 秒）
    if (t > kMaxCheckRelativeTime) {
      break;
    }
    double lon_v = p.v();
    // -0.1  40
    if (!WithinRange(lon_v, FLAGS_speed_lower_bound, FLAGS_speed_upper_bound)) {
      ADEBUG << "Velocity at relative time " << t
             << " exceeds bound, value: " << lon_v << ", bound ["
             << FLAGS_speed_lower_bound << ", " << FLAGS_speed_upper_bound
             << "].";
      return Result::LON_VELOCITY_OUT_OF_BOUND;
    }

    double lon_a = p.a();
    // -6    4
    if (!WithinRange(lon_a, FLAGS_longitudinal_acceleration_lower_bound,
                     FLAGS_longitudinal_acceleration_upper_bound)) {
      ADEBUG << "Longitudinal acceleration at relative time " << t
             << " exceeds bound, value: " << lon_a << ", bound ["
             << FLAGS_longitudinal_acceleration_lower_bound << ", "
             << FLAGS_longitudinal_acceleration_upper_bound << "].";
      return Result::LON_ACCELERATION_OUT_OF_BOUND;
    }

    double kappa = p.path_point().kappa();
    // 0.1979
    if (!WithinRange(kappa, -FLAGS_kappa_bound, FLAGS_kappa_bound)) {
      ADEBUG << "Kappa at relative time " << t
             << " exceeds bound, value: " << kappa << ", bound ["
             << -FLAGS_kappa_bound << ", " << FLAGS_kappa_bound << "].";
      return Result::CURVATURE_OUT_OF_BOUND;
    }
  }
  // 逐段检查高阶导数（加加速度 jerk、横向加速度）
  for (size_t i = 1; i < trajectory.NumOfPoints(); ++i) {
    const auto& p0 = trajectory.TrajectoryPointAt(static_cast<uint32_t>(i - 1));
    const auto& p1 = trajectory.TrajectoryPointAt(static_cast<uint32_t>(i));

    if (p1.relative_time() > kMaxCheckRelativeTime) {
      break;
    }

    double t = p0.relative_time();
    // 从第 1 个点开始，与前一个点组成线段，计算变化率（需要时间差 dt）
    double dt = p1.relative_time() - p0.relative_time();
    double d_lon_a = p1.a() - p0.a();
    // Jerk（加加速度） = 加速度变化率 = (a1 - a0) / dt（单位：m/s³）
    double lon_jerk = d_lon_a / dt;
    // -4    2
    if (!WithinRange(lon_jerk, FLAGS_longitudinal_jerk_lower_bound,
                     FLAGS_longitudinal_jerk_upper_bound)) {
      ADEBUG << "Longitudinal jerk at relative time " << t
             << " exceeds bound, value: " << lon_jerk << ", bound ["
             << FLAGS_longitudinal_jerk_lower_bound << ", "
             << FLAGS_longitudinal_jerk_upper_bound << "].";
      return Result::LON_JERK_OUT_OF_BOUND;
    }
    // 横向加速度公式：a_lat = v² × κ（由圆周运动公式 a = v² / r，且 κ = 1/r 推导）
    double lat_a = p1.v() * p1.v() * p1.path_point().kappa();
    // 4
    if (!WithinRange(lat_a, -FLAGS_lateral_acceleration_bound,
                     FLAGS_lateral_acceleration_bound)) {
      ADEBUG << "Lateral acceleration at relative time " << t
             << " exceeds bound, value: " << lat_a << ", bound ["
             << -FLAGS_lateral_acceleration_bound << ", "
             << FLAGS_lateral_acceleration_bound << "].";
      return Result::LAT_ACCELERATION_OUT_OF_BOUND;
    }

    // TODO(zhangyajia): this is temporarily disabled
    // due to low quality reference line.
    /**
    double d_lat_a = p1.v() * p1.v() * p1.path_point().kappa() -
                     p0.v() * p0.v() * p0.path_point().kappa();
    // 横向 jerk = (a_lat1 - a_lat0) / dt
    // 当前被禁用，原因：参考线质量不高，导致 kappa 抖动大，误报 jerk 超限
    double lat_jerk = d_lat_a / dt;
    // 4
    if (!WithinRange(lat_jerk, -FLAGS_lateral_jerk_bound,
                     FLAGS_lateral_jerk_bound)) {
      ADEBUG << "Lateral jerk at relative time " << t
             << " exceeds bound, value: " << lat_jerk << ", bound ["
             << -FLAGS_lateral_jerk_bound << ", " << FLAGS_lateral_jerk_bound
             << "].";
      return Result::LAT_JERK_OUT_OF_BOUND;
    }
    **/
  }

  return Result::VALID;
}

}  // namespace planning
}  // namespace apollo
