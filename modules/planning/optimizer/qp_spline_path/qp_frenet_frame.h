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
 * @file frenet_frame.h
 * @brief: natural coordinate system
 **/

#ifndef BAIDU_IDG_HOUSTON_OPTIMIZER_QP_SPLINE_PATH_OPTIMIZER_QP_FRENET_FRAME_H_
#define BAIDU_IDG_HOUSTON_OPTIMIZER_QP_SPLINE_PATH_OPTIMIZER_QP_FRENET_FRAME_H_

#include <Eigen/Core>
#include <memory>
#include "common/em_planning_data.h"
#include "common/environment.h"
#include "common/obstacle.h"
#include "common/speed/speed_data.h"
#include "reference_line/reference_line.h"

namespace apollo {
namespace planning {

namespace {
constexpr double kEpsilontol = 1e-6;
}

class QpFrenetFrame {
 public:
  QpFrenetFrame() = default;

  ErrorCode init(const Environment& environment,
                 const ReferenceLine& reference_line,
                 const DecisionData& decision_data, const SpeedData& speed_data,
                 const FrenetFramePoint& init_frenet_point,
                 const double start_s, const double end_s,
                 const double time_resolution, const std::size_t num_points);

  const ReferenceLine* reference_line() const;

  double feasible_longitudinal_upper_bound() const;

  bool get_overall_bound(const double s,
                         std::pair<double, double>* const bound) const;

  bool get_map_bound(const double s,
                     std::pair<double, double>* const bound) const;

  bool get_static_obstacle_bound(const double s,
                                 std::pair<double, double>* const bound) const;

  bool get_dynamic_obstacle_bound(const double s,
                                  std::pair<double, double>* const bound) const;

 private:
  bool find_longitudinal_distance(const double time,
                                  SpeedPoint* const speed_point);

  bool calcualate_discretized_veh_loc();

  bool mapping_dynamic_obstacle_with_decision(const Obstacle& obstacle);

  bool mapping_static_obstacle_with_decision(const Obstacle& obstacle);

  ErrorCode mapping_polygon(
      const std::vector<::adu::common::math::Vec2d>& corners,
      const double buffer, const bool nudge_side,
      std::vector<std::pair<double, double>>* const bound_map);

  // nudge_side > 0 update upper bound, nudge_side < 0 update lower_bound
  // nudge_side == 0 out of bound
  ErrorCode map_line(const SLPoint& start, const SLPoint& end,
                     const int nudge_side,
                     std::vector<std::pair<double, double>>* const constraint);

  std::pair<double, double> map_lateral_constraint(const SLPoint& start,
                                                   const SLPoint& end,
                                                   const int nudge_side,
                                                   const double s_start,
                                                   const double s_end);

  std::pair<std::size_t, std::size_t> find_interval(const double start,
                                                    const double end) const;

  bool calculate_hd_map_bound();

  bool calculate_static_obstacle_bound();

  bool calculate_dynamic_obstacle_bound();

  bool get_bound(const double s,
                 const std::vector<std::pair<double, double>>& bound_map,
                 std::pair<double, double>* const bound) const;

  std::size_t find_index(const double s) const;

 private:
  const ReferenceLine* _reference_line = nullptr;

  const Environment* _environment = nullptr;

  const SpeedData* _speed_profile = nullptr;

  const DecisionData* _decision_data = nullptr;

  ::adu::common::config::VehicleParam _veh_config;
  FrenetFramePoint _init_frenet_point;

  double _feasible_longitudinal_upper_bound = 0.0;
  double _start_s = 0.0;
  double _end_s = 0.0;
  double _time_resolution = 0.1;

  std::vector<double> _evaluated_knots;
  std::vector<SpeedPoint> _discretized_veh_loc;
  std::vector<std::pair<double, double>> _hdmap_bound;
  std::vector<std::pair<double, double>> _static_obstacle_bound;
  std::vector<std::pair<double, double>> _dynamic_obstacle_bound;
};
}  // namespace planning
}  // namespace apollo

#endif  // BAIDU_IDG_HOUSTON_OPTIMIZER_QP_SPLINE_PATH_OPTIMIZER_QP_FRENET_FRAME_H_
