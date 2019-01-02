/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/tasks/optimizers/qp_piecewise_jerk_path/qp_piecewise_jerk_path_optimizer.h"

#include <algorithm>

#include "modules/common/time/time.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;

namespace {
std::vector<std::pair<double, double>>::iterator min_pair_first(
    std::vector<std::pair<double, double>>::iterator begin,
    std::vector<std::pair<double, double>>::iterator end) {
  return std::min_element(begin, end, [](const std::pair<double, double>& lhs,
                                         const std::pair<double, double>& rhs) {
    return lhs.first < rhs.first;
  });
}

std::vector<std::pair<double, double>>::iterator max_pair_second(
    std::vector<std::pair<double, double>>::iterator begin,
    std::vector<std::pair<double, double>>::iterator end) {
  return std::max_element(begin, end, [](const std::pair<double, double>& lhs,
                                         const std::pair<double, double>& rhs) {
    return lhs.second < rhs.second;
  });
}

void assign_pair_first(std::vector<std::pair<double, double>>::iterator begin,
                       std::vector<std::pair<double, double>>::iterator end,
                       double first) {
  for (auto iter = begin; iter != end; ++iter) {
    iter->first = first;
  }
}

void assign_pair_second(std::vector<std::pair<double, double>>::iterator begin,
                        std::vector<std::pair<double, double>>::iterator end,
                        double second) {
  for (auto iter = begin; iter != end; ++iter) {
    iter->second = second;
  }
}

}  // namespace

QpPiecewiseJerkPathOptimizer::QpPiecewiseJerkPathOptimizer(
    const TaskConfig& config)
    : PathOptimizer(config) {
  SetName("QpPiecewiseJerkPathOptimizer");
  CHECK(config_.has_qp_piecewise_jerk_path_config());
}

std::vector<std::tuple<double, double, double>>
QpPiecewiseJerkPathOptimizer::GetLateralBounds(
    const SLBoundary& adc_sl, const common::FrenetFramePoint& frenet_point,
    const double qp_delta_s, double path_length,
    const ReferenceLine& reference_line,
    const std::vector<const Obstacle*>& obstacles) {
  const auto& qp_config = config_.qp_piecewise_jerk_path_config();
  int size = std::max(2, static_cast<int>(path_length / qp_delta_s));
  const double buffered_adc_width =
      adc_sl.end_l() - adc_sl.start_l() + qp_config.lateral_buffer();
  std::vector<std::pair<double, double>> lateral_bounds(size);
  double start_s = frenet_point.s();

  // expand by road'sl or lane's l
  double accumulated_s = start_s;
  for (size_t i = 0; i < lateral_bounds.size();
       ++i, accumulated_s += qp_delta_s) {
    double left = 0.0;
    double right = 0.0;
    reference_line.GetLaneWidth(accumulated_s, &left, &right);
    bool adc_off_left = adc_sl.end_l() > left;  // adc is at left side of lane
    bool adc_off_right = adc_sl.start_l() < -right;
    // when ADC is not inside lane, use the min of road width and adc's l
    if (adc_off_left || adc_off_right) {
      // adc is off the lane.
      double road_left = 0.0;
      double road_right = 0.0;
      reference_line.GetRoadWidth(accumulated_s, &road_left, &road_right);
      if (adc_off_left) {  // adc is on left side of lane
        lateral_bounds[i].first = -right;
        lateral_bounds[i].second = std::min(adc_sl.end_l(), road_left);
      } else {  // adc is on right side of road
        lateral_bounds[i].first = std::max(adc_sl.start_l(), -road_right);
        lateral_bounds[i].second = left;
      }
    } else {  // use the lane's width
      lateral_bounds[i].first = -right;
      lateral_bounds[i].second = left;
    }
  }

  // shrink bounds by obstacles.
  auto find_s_iter = [&](double s) {
    int index = static_cast<int>(s / qp_delta_s);
    if (index < 0) {
      index = 0;
    }
    if (index > static_cast<int>(lateral_bounds.size())) {
      index = static_cast<int>(lateral_bounds.size());
    }
    return lateral_bounds.begin() + index;
  };

  for (const auto* obstacle :
       reference_line_info_->path_decision()->obstacles().Items()) {
    // only takes care of static obstacles
    if (!obstacle->IsStatic()) {
      continue;
    }
    // ignore obstacles that are not in longitudinal range.
    const auto& obstacle_sl = obstacle->PerceptionSLBoundary();
    if (obstacle_sl.end_s() < start_s ||
        obstacle_sl.start_s() > accumulated_s) {
      continue;
    }
    auto start_iter = find_s_iter(obstacle_sl.start_s() - start_s);
    auto end_iter = find_s_iter(obstacle_sl.end_s() - start_s);
    if (end_iter != lateral_bounds.end()) {  // end is one pass the last one
      ++end_iter;
    }
    if (start_iter == lateral_bounds.end()) {
      continue;
    }
    double l_lower = max_pair_second(start_iter, end_iter)->first;
    double l_upper = min_pair_first(start_iter, end_iter)->second;
    // ignore obstacles that are not in lateral range
    if (obstacle_sl.start_l() > l_upper || obstacle_sl.end_l() < l_lower) {
      continue;
    }

    // handle parallel obstacles
    if (obstacle_sl.end_s() < adc_sl.end_s()) {
      if (obstacle_sl.start_l() > adc_sl.end_l()) {  // obstacle at left side
        assign_pair_second(start_iter, end_iter, obstacle_sl.start_l());
      } else {  // obstacle at right side
        assign_pair_first(start_iter, end_iter, obstacle_sl.end_l());
      }
      continue;
    }

    // handle general obstacles
    double left_remain = l_upper - obstacle_sl.end_l();
    double right_remain = -l_lower + obstacle_sl.start_l();
    if (left_remain > buffered_adc_width) {  // can pass at left side
      l_lower = std::max(l_lower, obstacle_sl.end_l());
    } else if (right_remain > buffered_adc_width) {  // can pass at right side
      l_upper = std::min(l_upper, obstacle_sl.start_l());
    } else {  // obstacle is blocking path
      if (obstacle_sl.start_l() * obstacle_sl.end_l() <
          0) {  // occupied ref line
        // path will not be affected by this obstacle. Very likely Path
        // Decider will stop for this obstacle.
      } else {  // reference line is not occupied by obstacle, try to bypass
        double road_left = 0.0;
        double road_right = 0.0;
        reference_line.GetRoadWidth(obstacle_sl.start_s(), &road_left,
                                    &road_right);
        if (obstacle_sl.start_l() >= 0) {  // pass from right side
          l_upper = obstacle_sl.start_l();
          l_lower =
              std::max(obstacle_sl.start_l() - buffered_adc_width, -road_right);
        } else {  // pass from left side
          l_upper =
              std::min(obstacle_sl.end_l() + buffered_adc_width, road_left);
          l_lower = obstacle_sl.end_l();
        }
      }
    }
    assign_pair_first(start_iter, end_iter, l_lower);
    assign_pair_second(start_iter, end_iter, l_upper);
  }

  std::vector<std::tuple<double, double, double>> lateral_bound_tuples;

  double s = 0.0;
  for (size_t i = 0; i < lateral_bounds.size(); ++i) {
    lateral_bound_tuples.emplace_back(std::make_tuple(
        s, std::get<0>(lateral_bounds[i]), std::get<1>(lateral_bounds[i])));
    s += qp_delta_s;
  }
  return lateral_bound_tuples;
}

Status QpPiecewiseJerkPathOptimizer::Process(
    const SpeedData& speed_data, const ReferenceLine& reference_line,
    const common::TrajectoryPoint& init_point, PathData* const path_data) {
  const auto frenet_point =
      reference_line.GetFrenetPoint(init_point.path_point());
  const auto& qp_config = config_.qp_piecewise_jerk_path_config();
  const auto& adc_sl = reference_line_info_->AdcSlBoundary();
  const double qp_delta_s = qp_config.qp_delta_s();
  const double path_length =
      std::fmax(qp_config.min_look_ahead_time() * init_point.v(),
                qp_config.min_look_ahead_distance());
  auto lateral_bounds = GetLateralBounds(
      adc_sl, frenet_point, qp_delta_s, path_length, reference_line,
      reference_line_info_->path_decision()->obstacles().Items());
  auto lateral_second_order_derivative_bounds =
      GetLateralSecondOrderDerivativeBounds(init_point, qp_delta_s);

  std::array<double, 3> init_lateral_state{frenet_point.l(), frenet_point.dl(),
                                           frenet_point.ddl()};
  const int n = static_cast<int>(path_length / qp_delta_s);

  std::array<double, 5> w = {
      qp_config.l_weight(),
      qp_config.dl_weight(),
      qp_config.ddl_weight(),
      qp_config.dddl_weight(),
      qp_config.guiding_line_weight(),
  };

  fem_1d_qp_.reset(new Fem1dExpandedJerkQpProblem());
  constexpr double kMaxLThirdOrderDerivative = 2.0;

  if (!fem_1d_qp_->Init(n, init_lateral_state, qp_delta_s, w,
                        kMaxLThirdOrderDerivative)) {
    std::string msg = "lateral qp optimizer failed";
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  auto start_time = std::chrono::system_clock::now();

  fem_1d_qp_->SetVariableBounds(lateral_bounds);
  fem_1d_qp_->SetVariableSecondOrderDerivativeBounds(
      lateral_second_order_derivative_bounds);

  bool success = fem_1d_qp_->Optimize();

  auto end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_time - start_time;
  ADEBUG << "Path Optimizer used time: " << diff.count() * 1000 << " ms.";

  if (!success) {
    AERROR << "lateral qp optimizer failed";
    return Status(ErrorCode::PLANNING_ERROR, "lateral qp optimizer failed");
  }
  fem_1d_qp_->SetOutputResolution(qp_config.path_output_resolution());

  std::vector<common::FrenetFramePoint> frenet_path;

  std::vector<common::FrenetFramePoint> frenet_frame_path;
  double accumulated_s = frenet_point.s();
  for (size_t i = 0; i < fem_1d_qp_->x().size(); ++i) {
    common::FrenetFramePoint frenet_frame_point;
    frenet_frame_point.set_s(accumulated_s);
    frenet_frame_point.set_l(fem_1d_qp_->x()[i]);
    frenet_frame_point.set_dl(fem_1d_qp_->x_derivative()[i]);
    frenet_frame_point.set_ddl(fem_1d_qp_->x_second_order_derivative()[i]);
    frenet_frame_path.push_back(std::move(frenet_frame_point));
    accumulated_s += qp_config.path_output_resolution();
  }

  path_data->SetReferenceLine(&reference_line);
  path_data->SetFrenetPath(FrenetFramePath(frenet_frame_path));

  return Status::OK();
}

std::vector<std::tuple<double, double, double>>
QpPiecewiseJerkPathOptimizer::GetLateralSecondOrderDerivativeBounds(
    const common::TrajectoryPoint& init_point, const double qp_delta_s) {
  std::vector<std::tuple<double, double, double>>
      lateral_second_order_derivative_bounds;
  constexpr double kMaxTolerableAcc = 0.5;
  constexpr double kMaxConstraintDist = 6.0;

  const double ddx = std::sqrt(kMaxTolerableAcc) / (1e-3 + init_point.v());

  for (double s = qp_delta_s; s < kMaxConstraintDist; s += qp_delta_s) {
    lateral_second_order_derivative_bounds.push_back(
        std::make_tuple(s, -ddx, ddx));
  }
  return lateral_second_order_derivative_bounds;
}

}  // namespace planning
}  // namespace apollo
