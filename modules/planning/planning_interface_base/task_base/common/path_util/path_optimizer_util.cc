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

#include "modules/planning/planning_interface_base/task_base/common/path_util/path_optimizer_util.h"

#include <utility>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/planning/planning_base/common/speed/speed_data.h"
#include "modules/planning/planning_base/common/trajectory1d/piecewise_jerk_trajectory1d.h"
#include "modules/planning/planning_base/common/util/print_debug_info.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_base/math/piecewise_jerk/piecewise_jerk_path_problem.h"
namespace apollo {
namespace planning {

FrenetFramePath PathOptimizerUtil::ToPiecewiseJerkPath(
    const std::vector<double>& x, const std::vector<double>& dx,
    const std::vector<double>& ddx, const double delta_s,
    const double start_s) {
  ACHECK(!x.empty());
  ACHECK(!dx.empty());
  ACHECK(!ddx.empty());

  PiecewiseJerkTrajectory1d piecewise_jerk_traj(x.front(), dx.front(),
                                                ddx.front());

  for (std::size_t i = 1; i < x.size(); ++i) {
    const auto dddl = (ddx[i] - ddx[i - 1]) / delta_s;
    piecewise_jerk_traj.AppendSegment(dddl, delta_s);
  }

  std::vector<common::FrenetFramePoint> frenet_frame_path;
  double accumulated_s = 0.0;
  while (accumulated_s < piecewise_jerk_traj.ParamLength()) {
    double l = piecewise_jerk_traj.Evaluate(0, accumulated_s);
    double dl = piecewise_jerk_traj.Evaluate(1, accumulated_s);
    double ddl = piecewise_jerk_traj.Evaluate(2, accumulated_s);

    common::FrenetFramePoint frenet_frame_point;
    frenet_frame_point.set_s(accumulated_s + start_s);
    frenet_frame_point.set_l(l);
    frenet_frame_point.set_dl(dl);
    frenet_frame_point.set_ddl(ddl);
    frenet_frame_path.push_back(std::move(frenet_frame_point));

    accumulated_s += FLAGS_trajectory_space_resolution;
  }

  return FrenetFramePath(std::move(frenet_frame_path));
}

double PathOptimizerUtil::EstimateJerkBoundary(const double vehicle_speed) {
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  const double axis_distance = veh_param.wheel_base();
  const double max_yaw_rate =
      veh_param.max_steer_angle_rate() / veh_param.steer_ratio();
  return max_yaw_rate / axis_distance / vehicle_speed;
}

std::vector<common::PathPoint>
PathOptimizerUtil::ConvertPathPointRefFromFrontAxeToRearAxe(
    const PathData& path_data) {
  std::vector<common::PathPoint> ret;
  double front_to_rear_axe_distance =
      apollo::common::VehicleConfigHelper::GetConfig()
          .vehicle_param()
          .wheel_base();
  for (auto path_point : path_data.discretized_path()) {
    common::PathPoint new_path_point = path_point;
    new_path_point.set_x(path_point.x() - front_to_rear_axe_distance *
                                              std::cos(path_point.theta()));
    new_path_point.set_y(path_point.y() - front_to_rear_axe_distance *
                                              std::sin(path_point.theta()));
    ret.push_back(new_path_point);
  }
  return ret;
}

// 该函数在 PathOptimizerUtil 命名空间中，用于对路径进行优化
bool PathOptimizerUtil::OptimizePath(
    // 初始状态：横向位置、横向速度、横向加速度
    const SLState& init_state,
    // 期望的终点状态：横向位置、横向速度、横向加速度
    const std::array<double, 3>& end_state,
    // 参考横向位置
    std::vector<double> l_ref,
    // 参考横向位置的权重
    std::vector<double> l_ref_weight,
    // 路径边界
    const PathBoundary& path_boundary,
    // 横向加速度的边界
    const std::vector<std::pair<double, double>>& ddl_bounds,
    // 横向加加速度的最大值
    double dddl_bound,
    // 路径优化的配置
    const PiecewiseJerkPathConfig& config,
    // 优化后的横向位置
    std::vector<double>* x,
    // 优化后的横向速度
    std::vector<double>* dx,
    // 优化后的横向加速度
    std::vector<double>* ddx) {
  // 路径点的数量
  const auto& lat_boundaries = path_boundary.boundary();
  const size_t kNumKnots = lat_boundaries.size();

  // 路径点之间的间隔
  double delta_s = path_boundary.delta_s();

  // 创建 PiecewiseJerkPathProblem 实例
  PiecewiseJerkPathProblem piecewise_jerk_problem(kNumKnots, delta_s,
                                                  init_state.second);

  // 用于调试的类，用于记录路径点
  PrintCurves print_curve;

  // 记录路径点
  for (size_t i = 0; i < kNumKnots; i++) {
    print_curve.AddPoint(path_boundary.label() + "_ref_l",
                         i * path_boundary.delta_s(), l_ref[i]);
    print_curve.AddPoint(path_boundary.label() + "_ref_l_weight",
                         i * path_boundary.delta_s(), l_ref_weight[i]);
    print_curve.AddPoint(path_boundary.label() + "_l_lower",
                         i * path_boundary.delta_s(), lat_boundaries[i].first);
    print_curve.AddPoint(path_boundary.label() + "_l_upper",
                         i * path_boundary.delta_s(), lat_boundaries[i].second);
    print_curve.AddPoint(path_boundary.label() + "_ddl_lower",
                         i * path_boundary.delta_s(), ddl_bounds[i].first);
    print_curve.AddPoint(path_boundary.label() + "_ddl_upper",
                         i * path_boundary.delta_s(), ddl_bounds[i].second);
  }

  // 记录初始状态
  print_curve.AddPoint(path_boundary.label() + "_opt_l", 0,
                       init_state.second[0]);
  print_curve.AddPoint(path_boundary.label() + "_opt_dl", 0,
                       init_state.second[1]);
  print_curve.AddPoint(path_boundary.label() + "_opt_ddl", 0,
                       init_state.second[2]);

  // TODO(Hongyi): 更新 end_state 设置
  std::array<double, 3U> end_state_weight = {config.weight_end_state_l(),
                                             config.weight_end_state_dl(),
                                             config.weight_end_state_ddl()};

  // 设置期望的终点状态
  piecewise_jerk_problem.set_end_state_ref(end_state_weight, end_state);

  // 设置参考横向位置
  piecewise_jerk_problem.set_x_ref(std::move(l_ref_weight), l_ref);

  // 用于调试：在此处使用 std::move
  piecewise_jerk_problem.set_weight_x(config.l_weight());
  piecewise_jerk_problem.set_weight_dx(config.dl_weight());
  piecewise_jerk_problem.set_weight_ddx(config.ddl_weight());
  piecewise_jerk_problem.set_weight_dddx(config.dddl_weight());

  // 设置缩放因子
  piecewise_jerk_problem.set_scale_factor({1.0, 10.0, 100.0});

  // 记录开始时间
  auto start_time = std::chrono::system_clock::now();

  // 设置横向位置的边界
  piecewise_jerk_problem.set_x_bounds(lat_boundaries);

  // 设置横向速度的边界
  piecewise_jerk_problem.set_dx_bounds(
      -config.lateral_derivative_bound_default(),
      config.lateral_derivative_bound_default());

  // 设置横向加速度的边界
  piecewise_jerk_problem.set_ddx_bounds(ddl_bounds);

  // 设置横向加加速度的最大值
  piecewise_jerk_problem.set_dddx_bound(dddl_bound);

  // 进行路径优化
  bool success = piecewise_jerk_problem.Optimize(config.max_iteration());

  // 记录结束时间
  auto end_time = std::chrono::system_clock::now();

  // 计算路径优化所用的时间
  std::chrono::duration<double> diff = end_time - start_time;
  ADEBUG << "路径优化器所用时间: " << diff.count() * 1000 << " ms.";

  // 如果路径优化失败，返回 false
  if (!success) {
    AERROR << path_boundary.label() << "分段三次曲线路径优化器失败";
    AINFO << "初始状态：s(" << init_state.first[0] << "," << init_state.first[1]
          << "," << init_state.first[2] << ") l (" << init_state.second[0]
          << "," << init_state.second[1] << "," << init_state.second[2];
    AINFO << "dx 边界: " << config.lateral_derivative_bound_default();
    AINFO << "jerk 边界: " << dddl_bound;
    print_curve.PrintToLog();
    return false;
  }

  // 记录优化后的路径点
  *x = piecewise_jerk_problem.opt_x();
  *dx = piecewise_jerk_problem.opt_dx();
  *ddx = piecewise_jerk_problem.opt_ddx();

  // 记录优化后的路径点
  for (size_t i = 0; i < kNumKnots; i++) {
    print_curve.AddPoint(path_boundary.label() + "_opt_l",
                         i * path_boundary.delta_s(), (*x)[i]);
    print_curve.AddPoint(path_boundary.label() + "_opt_dl",
                         i * path_boundary.delta_s(), (*dx)[i]);
    print_curve.AddPoint(path_boundary.label() + "_opt_ddl",
                         i * path_boundary.delta_s(), (*ddx)[i]);
  }

  // 打印并记录路径点
  print_curve.PrintToLog();

  // 如果路径优化成功，返回 true
  return true;
}

void PathOptimizerUtil::UpdatePathRefWithBound(
    const PathBoundary& path_boundary, double weight,
    std::vector<double>* ref_l, std::vector<double>* weight_ref_l) {
  ref_l->resize(path_boundary.size());
  weight_ref_l->resize(path_boundary.size());
  for (size_t i = 0; i < ref_l->size(); i++) {
    if (path_boundary[i].l_lower.type == BoundType::OBSTACLE ||
        path_boundary[i].l_upper.type == BoundType::OBSTACLE) {
      ref_l->at(i) =
          (path_boundary[i].l_lower.l + path_boundary[i].l_upper.l) / 2.0;
      weight_ref_l->at(i) = weight;
    } else {
      weight_ref_l->at(i) = 0;
    }
  }
}

void PathOptimizerUtil::CalculateAccBound(
    const PathBoundary& path_boundary, const ReferenceLine& reference_line,
    std::vector<std::pair<double, double>>* ddl_bounds) {
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  const double lat_acc_bound =
      std::tan(veh_param.max_steer_angle() / veh_param.steer_ratio()) /
      veh_param.wheel_base();
  size_t path_boundary_size = path_boundary.boundary().size();
  for (size_t i = 0; i < path_boundary_size; ++i) {
    double s = static_cast<double>(i) * path_boundary.delta_s() +
               path_boundary.start_s();
    double kappa = reference_line.GetNearestReferencePoint(s).kappa();
    ddl_bounds->emplace_back(-lat_acc_bound - kappa, lat_acc_bound - kappa);
  }
}

}  // namespace planning
}  // namespace apollo
