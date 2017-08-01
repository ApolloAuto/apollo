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

#ifndef MODULES_PLANNING_OPTIMIZER_QP_SPLINE_PATH_QP_FRENET_FRAME_H_
#define MODULES_PLANNING_OPTIMIZER_QP_SPLINE_PATH_QP_FRENET_FRAME_H_

#include <memory>
#include <utility>
#include <vector>

#include "Eigen/Core"

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/planning_data.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/reference_line/reference_line.h"

namespace apollo {
namespace planning {

class QpFrenetFrame {
 public:
  QpFrenetFrame() = default;

  bool Init(const ReferenceLine& reference_line,
            const DecisionData& decision_data, const SpeedData& speed_data,
            const common::FrenetFramePoint& init_frenet_point,
            const double start_s, const double end_s,
            const double time_resolution, const uint32_t num_points);

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
                                  common::SpeedPoint* const speed_point);

  bool calcualate_discretized_veh_loc();

  bool mapping_dynamic_obstacle_with_decision(const Obstacle& obstacle);

  bool mapping_static_obstacle_with_decision(const Obstacle& obstacle);

  bool mapping_polygon(const std::vector<common::math::Vec2d>& corners,
                       const double buffer, const bool nudge_side,
                       std::vector<std::pair<double, double>>* const bound_map);

  // nudge_side > 0 update upper bound, nudge_side < 0 update lower_bound
  // nudge_side == 0 out of bound
  bool map_line(const common::SLPoint& start, const common::SLPoint& end,
                const int nudge_side,
                std::vector<std::pair<double, double>>* const constraint);

  std::pair<double, double> map_lateral_constraint(const common::SLPoint& start,
                                                   const common::SLPoint& end,
                                                   const int nudge_side,
                                                   const double s_start,
                                                   const double s_end);

  std::pair<uint32_t, uint32_t> find_interval(const double start,
                                              const double end) const;

  bool calculate_hd_map_bound();

  bool calculate_static_obstacle_bound();

  bool calculate_dynamic_obstacle_bound();

  bool get_bound(const double s,
                 const std::vector<std::pair<double, double>>& bound_map,
                 std::pair<double, double>* const bound) const;

  uint32_t find_index(const double s) const;

  void clear_data();

 private:
  const ReferenceLine* _reference_line = nullptr;

  const SpeedData* _speed_profile = nullptr;

  const DecisionData* _decision_data = nullptr;

  common::VehicleParam _vehicle_param;
  common::FrenetFramePoint _init_frenet_point;

  double _feasible_longitudinal_upper_bound = 0.0;
  double _start_s = 0.0;
  double _end_s = 0.0;
  double _time_resolution = 0.1;

  std::vector<double> _evaluated_knots;
  std::vector<common::SpeedPoint> _discretized_veh_loc;
  std::vector<std::pair<double, double>> _hdmap_bound;
  std::vector<std::pair<double, double>> _static_obstacle_bound;
  std::vector<std::pair<double, double>> _dynamic_obstacle_bound;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_OPTIMIZER_QP_SPLINE_PATH_QP_FRENET_FRAME_H_
