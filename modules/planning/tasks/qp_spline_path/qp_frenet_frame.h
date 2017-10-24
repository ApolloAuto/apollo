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
 * @file qp_frenet_frame.h
 * @brief: natural coordinate system
 **/

#ifndef MODULES_PLANNING_TASKS_QP_SPLINE_PATH_QP_FRENET_FRAME_H_
#define MODULES_PLANNING_TASKS_QP_SPLINE_PATH_QP_FRENET_FRAME_H_

#include <memory>
#include <utility>
#include <vector>

#include "Eigen/Core"

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/planning_internal.pb.h"

#include "modules/planning/common/path_obstacle.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/reference_line/reference_line.h"

namespace apollo {
namespace planning {

class QpFrenetFrame {
 public:
  QpFrenetFrame(const ReferenceLine& reference_line,
                const SpeedData& speed_data,
                const common::FrenetFramePoint& init_frenet_point,
                const double time_resolution,
                const std::vector<double>& evaluated_s);
  virtual ~QpFrenetFrame() = default;

  bool Init(const std::vector<const PathObstacle*>& path_obstacles);

  void LogQpBound(apollo::planning_internal::Debug* planning_debug);

  bool GetMapBound(const double s,
                   std::pair<double, double>* const bound) const;

  bool GetStaticObstacleBound(const double s,
                              std::pair<double, double>* const bound) const;

  bool GetDynamicObstacleBound(const double s,
                               std::pair<double, double>* const bound) const;

 private:
  bool CalculateDiscretizedVehicleLocation();

  bool MapDynamicObstacleWithDecision(const PathObstacle& path_obstacle);

  bool MapStaticObstacleWithDecision(const PathObstacle& path_obstacle);

  bool MapNudgePolygon(const common::math::Polygon2d& polygon,
                       const ObjectNudge& nudge,
                       std::vector<std::pair<double, double>>* const bound_map);

  bool MapNudgeLine(const common::SLPoint& start, const common::SLPoint& end,
                    const ObjectNudge::Type type,
                    std::vector<std::pair<double, double>>* const constraint);

  std::pair<double, double> MapLateralConstraint(
      const common::SLPoint& start, const common::SLPoint& end,
      const ObjectNudge::Type nudge_type, const double s_start,
      const double s_end);

  std::pair<uint32_t, uint32_t> FindInterval(const double start,
                                             const double end) const;

  bool CalculateHDMapBound();

  bool CalculateObstacleBound(
      const std::vector<const PathObstacle*>& path_obstacles);

  bool GetBound(const double s,
                const std::vector<std::pair<double, double>>& map_bound,
                std::pair<double, double>* const bound) const;

  uint32_t FindIndex(const double s) const;

 private:
  const ReferenceLine& reference_line_;
  const SpeedData& speed_data_;

  common::VehicleParam vehicle_param_;
  common::FrenetFramePoint init_frenet_point_;

  double feasible_longitudinal_upper_bound_ = 0.0;
  double start_s_ = 0.0;
  double end_s_ = 0.0;
  double time_resolution_ = 0.1;

  std::vector<double> evaluated_s_;
  std::vector<common::SpeedPoint> discretized_vehicle_location_;
  std::vector<std::pair<double, double>> hdmap_bound_;
  std::vector<std::pair<double, double>> static_obstacle_bound_;
  std::vector<std::pair<double, double>> dynamic_obstacle_bound_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_QP_SPLINE_PATH_QP_FRENET_FRAME_H_
