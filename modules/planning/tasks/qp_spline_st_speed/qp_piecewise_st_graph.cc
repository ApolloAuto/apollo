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
 * @file qp_piecewise_st_graph.cc
 **/

#include "modules/planning/tasks/qp_spline_st_speed/qp_piecewise_st_graph.h"

#include <algorithm>
#include <limits>
#include <string>
#include <utility>

#include "modules/common/log.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleParam;
using apollo::planning_internal::STGraphDebug;

QpPiecewiseStGraph::QpPiecewiseStGraph(
    const QpSplineStSpeedConfig& qp_spline_st_speed_config,
    const VehicleParam& veh_param)
    : qp_spline_st_speed_config_(qp_spline_st_speed_config),
      t_evaluated_resolution_(
          qp_spline_st_speed_config_.total_time() /
          qp_spline_st_speed_config_.number_of_evaluated_graph_t()) {
  Init();
}

void QpPiecewiseStGraph::Init() {
  // init evaluated t positions
  double curr_t = t_evaluated_resolution_;
  for (uint32_t i = 0;
       i < qp_spline_st_speed_config_.number_of_evaluated_graph_t(); ++i) {
    t_evaluated_.push_back(curr_t);
    curr_t += t_evaluated_resolution_;
  }
}

void QpPiecewiseStGraph::SetDebugLogger(
    planning_internal::STGraphDebug* st_graph_debug) {
  if (st_graph_debug) {
    st_graph_debug->Clear();
    st_graph_debug_ = st_graph_debug;
  }
}

Status QpPiecewiseStGraph::Search(
    const StGraphData& st_graph_data, SpeedData* const speed_data,
    const std::pair<double, double>& accel_bound) {
  cruise_.clear();

  // reset piecewise linear generator
  generator_.reset(new PiecewiseLinearGenerator(
      qp_spline_st_speed_config_.number_of_evaluated_graph_t(),
      t_evaluated_resolution_));

  // start to search for best st points
  init_point_ = st_graph_data.init_point();

  // modified total path length
  if (st_graph_data.path_data_length() <
      qp_spline_st_speed_config_.total_path_length()) {
    qp_spline_st_speed_config_.set_total_path_length(
        st_graph_data.path_data_length());
  }

  if (!ApplyConstraint(st_graph_data.init_point(), st_graph_data.speed_limit(),
                       st_graph_data.st_boundaries(), accel_bound)
           .ok()) {
    const std::string msg = "Apply constraint failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!ApplyKernel(st_graph_data.st_boundaries(), st_graph_data.speed_limit())
           .ok()) {
    const std::string msg = "Apply kernel failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!Solve().ok()) {
    const std::string msg = "Solve qp problem failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // extract output
  speed_data->Clear();
  const auto& res = generator_->params();
  speed_data->AppendSpeedPoint(0.0, 0.0, init_point_.v(), init_point_.a(), 0.0);

  double s = 0.0;
  double v = 0.0;
  double a = 0.0;

  double time = t_evaluated_resolution_;
  double dt = t_evaluated_resolution_;

  for (int i = 0; i < res.rows(); ++i, time += t_evaluated_resolution_) {
    s = res(i, 0);
    if (i == 0) {
      v = s / dt;
      a = (v - init_point_.v()) / dt;
    } else {
      const double curr_v = (s - res(i - 1, 0)) / dt;
      a = (curr_v - v) / dt;
      v = curr_v;
    }
    speed_data->AppendSpeedPoint(s, time, v, a, 0.0);
  }
  return Status::OK();
}

Status QpPiecewiseStGraph::ApplyConstraint(
    const common::TrajectoryPoint& init_point, const SpeedLimit& speed_limit,
    const std::vector<StBoundary>& boundaries,
    const std::pair<double, double>& accel_bound) {
  // TODO(Lianliang): implement this function.
  return Status::OK();
}

Status QpPiecewiseStGraph::ApplyKernel(
    const std::vector<StBoundary>& boundaries, const SpeedLimit& speed_limit) {
  // TODO(Lianliang): implement this function.
  return Status::OK();
}

Status QpPiecewiseStGraph::AddFollowReferenceLineKernel(
    const std::vector<StBoundary>& boundaries, const double weight) {
  // TODO(Lianliang): implement this function.
  return Status::OK();
}

Status QpPiecewiseStGraph::GetSConstraintByTime(
    const std::vector<StBoundary>& boundaries, const double time,
    const double total_path_s, double* const s_upper_bound,
    double* const s_lower_bound) const {
  // TODO(Lianliang): implement this function.
  return Status::OK();
}

Status QpPiecewiseStGraph::EstimateSpeedUpperBound(
    const common::TrajectoryPoint& init_point, const SpeedLimit& speed_limit,
    std::vector<double>* speed_upper_bound) const {
  // TODO(Lianliang): implement this function.
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
