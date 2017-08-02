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
 * @file qp_spline_st_graph.h
 **/

#ifndef MODULES_PLANNING_OPTIMIZER_QP_SPLINE_ST_SPEED_QP_SPLINE_ST_GRAPH_H_
#define MODULES_PLANNING_OPTIMIZER_QP_SPLINE_ST_SPEED_QP_SPLINE_ST_GRAPH_H_

#include <memory>
#include <vector>

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/qp_spline_st_speed_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/common/util/string_util.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/planning_util.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/math/smoothing_spline/spline_1d_generator.h"
#include "modules/planning/optimizer/st_graph/st_graph_boundary.h"
#include "modules/planning/optimizer/st_graph/st_graph_data.h"

namespace apollo {
namespace planning {

class QpSplineStGraph {
 public:
  QpSplineStGraph(const QpSplineStSpeedConfig& qp_config,
                  const apollo::common::VehicleParam& veh_param);

  common::Status Search(const StGraphData& st_graph_data,
                        const PathData& path_data, SpeedData* const speed_data);

 private:
  void Init();

  // apply st graph constraint
  common::Status ApplyConstraint(
      const std::vector<StGraphBoundary>& boundaries);

  // apply objective function
  common::Status ApplyKernel(const SpeedLimit& speed_limit);

  // solve
  common::Status Solve();

  // extract upper lower bound for constraint;
  common::Status GetSConstraintByTime(
      const std::vector<StGraphBoundary>& boundaries, const double time,
      const double total_path_s, double* const s_upper_bound,
      double* const s_lower_bound) const;

  // generate reference speed profile
  // common::Status ApplyReferenceSpeedProfile();

  common::Status AddFollowReferenceLineKernel(
      const StGraphBoundary& follow_boundary);

 private:
  // qp st configuration
  QpSplineStSpeedConfig qp_spline_st_speed_config_;

  // initial status
  common::TrajectoryPoint init_point_;

  // solver
  std::unique_ptr<Spline1dGenerator> spline_generator_ = nullptr;

  // time resolution
  double time_resolution_ = 0.0;

  // t_knots
  std::vector<double> t_knots_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_OPTIMIZER_QP_SPLINE_ST_SPEED_QP_SPLINE_ST_GRAPH_H_
