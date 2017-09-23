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

Status QpPiecewiseStGraph::AddCruiseReferenceLineKernel(
    const SpeedLimit& speed_limit, const double weight) {
  auto* ref_kernel = generator_->mutable_kernel();
  if (speed_limit.speed_limit_points().size() == 0) {
    std::string msg = "Fail to apply_kernel due to empty speed limits.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  double dist_ref = 0.0;
  std::vector<uint32_t> index_list;

  for (uint32_t i = 0; i < t_evaluated_.size(); ++i) {
    index_list.push_back(i);
    dist_ref +=
        t_evaluated_resolution_ * speed_limit.GetSpeedLimitByS(dist_ref);
    cruise_.push_back(dist_ref);
  }
  if (st_graph_debug_) {
    auto kernel_cruise_ref = st_graph_debug_->mutable_kernel_cruise_ref();
    kernel_cruise_ref->mutable_t()->Add(t_evaluated_[0]);
    kernel_cruise_ref->mutable_cruise_line_s()->Add(dist_ref);
    for (uint32_t i = 1; i < t_evaluated_.size(); ++i) {
      kernel_cruise_ref->mutable_t()->Add(t_evaluated_[i]);
      kernel_cruise_ref->mutable_cruise_line_s()->Add(cruise_[i]);
    }
  }
  DCHECK_EQ(t_evaluated_.size(), cruise_.size());

  for (std::size_t i = 0; i < t_evaluated_.size(); ++i) {
    ADEBUG << "Cruise Ref S: " << cruise_[i]
           << " Relative time: " << t_evaluated_[i] << std::endl;
  }

  if (t_evaluated_.size() > 0) {
    ref_kernel->AddReferenceLineKernelMatrix(
        index_list, cruise_,
        weight * t_evaluated_.size() / qp_spline_st_speed_config_.total_time());
  }

  ref_kernel->AddRegularization(
      qp_spline_st_speed_config_.regularization_weight());
  return Status::OK();
}

Status QpPiecewiseStGraph::AddFollowReferenceLineKernel(
    const std::vector<StBoundary>& boundaries, const double weight) {
  auto* follow_kernel = generator_->mutable_kernel();
  std::vector<double> ref_s;
  std::vector<double> filtered_evaluate_t;
  std::vector<uint32_t> index_list;
  for (size_t i = 0; i < t_evaluated_.size(); ++i) {
    const double curr_t = t_evaluated_[i];
    double s_min = std::numeric_limits<double>::infinity();
    bool success = false;
    for (const auto& boundary : boundaries) {
      if (boundary.boundary_type() != StBoundary::BoundaryType::FOLLOW) {
        continue;
      }
      if (curr_t < boundary.min_t() || curr_t > boundary.max_t()) {
        continue;
      }
      double s_upper = 0.0;
      double s_lower = 0.0;
      if (boundary.GetUnblockSRange(curr_t, &s_upper, &s_lower)) {
        success = true;
        s_min = std::min(s_min,
                         s_upper - boundary.characteristic_length() -
                             qp_spline_st_speed_config_.follow_drag_distance());
      }
    }
    if (success && s_min < cruise_[i]) {
      filtered_evaluate_t.push_back(curr_t);
      ref_s.push_back(s_min);
      index_list.push_back(i);
      if (st_graph_debug_) {
        auto kernel_follow_ref = st_graph_debug_->mutable_kernel_follow_ref();
        kernel_follow_ref->mutable_t()->Add(curr_t);
        kernel_follow_ref->mutable_follow_line_s()->Add(s_min);
      }
    }
  }
  DCHECK_EQ(filtered_evaluate_t.size(), ref_s.size());

  if (!ref_s.empty()) {
    follow_kernel->AddReferenceLineKernelMatrix(
        index_list, ref_s,
        weight * t_evaluated_.size() / qp_spline_st_speed_config_.total_time());
  }

  for (std::size_t i = 0; i < filtered_evaluate_t.size(); ++i) {
    ADEBUG << "Follow Ref S: " << ref_s[i]
           << " Relative time: " << filtered_evaluate_t[i] << std::endl;
  }
  return Status::OK();
}

Status QpPiecewiseStGraph::GetSConstraintByTime(
    const std::vector<StBoundary>& boundaries, const double time,
    const double total_path_s, double* const s_upper_bound,
    double* const s_lower_bound) const {
  *s_upper_bound = total_path_s;

  for (const StBoundary& boundary : boundaries) {
    double s_upper = 0.0;
    double s_lower = 0.0;

    if (!boundary.GetUnblockSRange(time, &s_upper, &s_lower)) {
      continue;
    }

    if (boundary.boundary_type() == StBoundary::BoundaryType::STOP ||
        boundary.boundary_type() == StBoundary::BoundaryType::FOLLOW ||
        boundary.boundary_type() == StBoundary::BoundaryType::YIELD) {
      *s_upper_bound = std::fmin(*s_upper_bound, s_upper);
    } else {
      DCHECK(boundary.boundary_type() == StBoundary::BoundaryType::OVERTAKE);
      *s_lower_bound = std::fmax(*s_lower_bound, s_lower);
    }
  }

  return Status::OK();
}

Status QpPiecewiseStGraph::EstimateSpeedUpperBound(
    const common::TrajectoryPoint& init_point, const SpeedLimit& speed_limit,
    std::vector<double>* speed_upper_bound) const {
  // TODO(Lianliang): define a QpStGraph class and move this function in it.
  DCHECK_NOTNULL(speed_upper_bound);

  speed_upper_bound->clear();

  // use v to estimate position: not accurate, but feasible in cyclic
  // processing. We can do the following process multiple times and use
  // previous cycle's results for better estimation.
  const double v = init_point.v();

  if (static_cast<double>(t_evaluated_.size() +
                          speed_limit.speed_limit_points().size()) <
      t_evaluated_.size() * std::log(static_cast<double>(
                                speed_limit.speed_limit_points().size()))) {
    uint32_t i = 0;
    uint32_t j = 0;
    const double kDistanceEpsilon = 1e-6;
    while (i < t_evaluated_.size() &&
           j + 1 < speed_limit.speed_limit_points().size()) {
      const double distance = v * t_evaluated_[i];
      if (fabs(distance - speed_limit.speed_limit_points()[j].first) <
          kDistanceEpsilon) {
        speed_upper_bound->push_back(
            speed_limit.speed_limit_points()[j].second);
        ++i;
        ADEBUG << "speed upper bound:" << speed_upper_bound->back();
      } else if (distance < speed_limit.speed_limit_points()[j].first) {
        ++i;
      } else if (distance <= speed_limit.speed_limit_points()[j + 1].first) {
        speed_upper_bound->push_back(speed_limit.GetSpeedLimitByS(distance));
        ADEBUG << "speed upper bound:" << speed_upper_bound->back();
        ++i;
      } else {
        ++j;
      }
    }

    for (uint32_t k = speed_upper_bound->size(); k < t_evaluated_.size(); ++k) {
      speed_upper_bound->push_back(FLAGS_planning_upper_speed_limit);
      ADEBUG << "speed upper bound:" << speed_upper_bound->back();
    }
  } else {
    auto cmp = [](const std::pair<double, double>& p1, const double s) {
      return p1.first < s;
    };

    const auto& speed_limit_points = speed_limit.speed_limit_points();
    for (const double t : t_evaluated_) {
      const double s = v * t;

      // NOTICE: we are using binary search here based on two assumptions:
      // (1) The s in speed_limit_points increase monotonically.
      // (2) The evaluated_t_.size() << number of speed_limit_points.size()
      //
      // If either of the two assumption is failed, a new algorithm must be
      // used
      // to replace the binary search.

      const auto& it = std::lower_bound(speed_limit_points.begin(),
                                        speed_limit_points.end(), s, cmp);
      if (it != speed_limit_points.end()) {
        speed_upper_bound->push_back(it->second);
      } else {
        speed_upper_bound->push_back(speed_limit_points.back().second);
      }
    }
  }

  const double kTimeBuffer = 2.0;
  const double kSpeedBuffer = 0.1;
  for (uint32_t k = 0; k < t_evaluated_.size() && t_evaluated_[k] < kTimeBuffer;
       ++k) {
    speed_upper_bound->at(k) =
        std::fmax(init_point_.v() + kSpeedBuffer, speed_upper_bound->at(k));
  }

  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
