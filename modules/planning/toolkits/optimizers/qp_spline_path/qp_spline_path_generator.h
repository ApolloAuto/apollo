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
 * @file qp_spline_path_generator.h
 **/

#ifndef MODULES_PLANNING_TASKS_QP_SPLINE_PATH_QP_SPLINE_PATH_GENERATOR_H_
#define MODULES_PLANNING_TASKS_QP_SPLINE_PATH_QP_SPLINE_PATH_GENERATOR_H_

#include <memory>
#include <string>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/planning_internal.pb.h"
#include "modules/planning/proto/qp_spline_path_config.pb.h"
#include "modules/planning/proto/sl_boundary.pb.h"

#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/path_decision.h"
#include "modules/planning/common/path_obstacle.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/math/smoothing_spline/spline_1d_generator.h"
#include "modules/planning/toolkits/optimizers/qp_spline_path/qp_frenet_frame.h"

namespace apollo {
namespace planning {

class QpSplinePathGenerator {
 public:
  QpSplinePathGenerator(Spline1dGenerator* spline_generator,
                        const ReferenceLine& reference_line,
                        const QpSplinePathConfig& qp_spline_path_config,
                        const SLBoundary& adc_sl_boundary);

  void SetDebugLogger(apollo::planning_internal::Debug* debug);

  void SetChangeLane(bool is_change_lane_path);

  bool Generate(const std::vector<const PathObstacle*>& path_obstacles,
                const SpeedData& speed_data,
                const common::TrajectoryPoint& init_point,
                const double boundary_extension, bool is_final_attempt,
                PathData* const path_data);

 private:
  bool CalculateFrenetPoint(const common::TrajectoryPoint& traj_point,
                            common::FrenetFramePoint* const sl_point);

  bool InitSpline(const double start_s, const double end_s);

  bool AddConstraint(const QpFrenetFrame& qp_frenet_frame,
                     const double boundary_extension);

  void AddKernel();

  void AddHistoryPathKernel();

  bool Solve();

 private:
  Spline1dGenerator* spline_generator_ = nullptr;
  apollo::planning_internal::Debug* planning_debug_ = nullptr;
  const ReferenceLine& reference_line_;
  const QpSplinePathConfig& qp_spline_path_config_;

  common::TrajectoryPoint init_trajectory_point_;
  common::FrenetFramePoint init_frenet_point_;

  std::vector<double> knots_;
  std::vector<double> evaluated_s_;

  const DiscretizedPath* last_discretized_path_ = nullptr;
  SLBoundary adc_sl_boundary_;
  bool is_change_lane_path_ = false;
  double ref_l_ = 0.0;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_QP_SPLINE_PATH_QP_SPLINE_PATH_GENERATOR_H_
