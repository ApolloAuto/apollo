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
 * @file
 **/

#include "modules/planning/tasks/st_graph/st_boundary_mapper.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/decision.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/log.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/file.h"
#include "modules/common/util/string_util.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::SLPoint;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleParam;
using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;

namespace {
constexpr double boundary_t_buffer = 0.1;
constexpr double boundary_s_buffer = 1.0;
}

StBoundaryMapper::StBoundaryMapper(const SLBoundary& adc_sl_boundary,
                                   const StBoundaryConfig& config,
                                   const ReferenceLine& reference_line,
                                   const PathData& path_data,
                                   const double planning_distance,
                                   const double planning_time)
    : adc_sl_boundary_(adc_sl_boundary),
      st_boundary_config_(config),
      reference_line_(reference_line),
      path_data_(path_data),
      vehicle_param_(
          common::VehicleConfigHelper::instance()->GetConfig().vehicle_param()),
      planning_distance_(planning_distance),
      planning_time_(planning_time) {}

Status StBoundaryMapper::GetGraphBoundary(PathDecision* path_decision) const {
  const auto& path_obstacles = path_decision->path_obstacles();
  if (planning_time_ < 0.0) {
    const std::string msg = "Fail to get params since planning_time_ < 0.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (path_data_.discretized_path().NumOfPoints() < 2) {
    AERROR << "Fail to get params because of too few path points. path points "
              "size: "
           << path_data_.discretized_path().NumOfPoints() << ".";
    return Status(ErrorCode::PLANNING_ERROR,
                  "Fail to get params because of too few path points");
  }

  PathObstacle* stop_obstacle = nullptr;
  ObjectDecisionType stop_decision;
  double min_stop_s = std::numeric_limits<double>::max();

  for (const auto* const_path_obstacle : path_obstacles.Items()) {
    auto* path_obstacle = path_decision->Find(const_path_obstacle->Id());
    StBoundary boundary;
    boundary.SetId(path_obstacle->Id());
    if (!path_obstacle->HasLongitudinalDecision()) {
      const auto ret = MapWithoutDecision(path_obstacle);
      if (!ret.ok()) {
        std::string msg = common::util::StrCat(
            "Fail to map obstacle ", path_obstacle->Id(), " without decision.");
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
      }
      continue;
    }
    const auto& decision = path_obstacle->LongitudinalDecision();
    if (decision.has_stop()) {
      const double stop_s = path_obstacle->perception_sl_boundary().start_s() +
                            decision.stop().distance_s();
      // this is a rough estimation based on reference line s, so that a large
      // buffer is used.
      constexpr double stop_buff = 1.0;
      if (stop_s + stop_buff < adc_sl_boundary_.end_s()) {
        AERROR << "Invalid stop decision. not stop at behind of current "
                  "position. stop_s : "
               << stop_s << ", and current adc_s is; "
               << adc_sl_boundary_.end_s();
        return Status(ErrorCode::PLANNING_ERROR, "invalid decision");
      }
      if (!stop_obstacle) {
        stop_obstacle = path_obstacle;
        stop_decision = decision;
        min_stop_s = stop_s;
      } else if (stop_s < min_stop_s) {
        stop_obstacle = path_obstacle;
        min_stop_s = stop_s;
        stop_decision = decision;
      }
    } else if (decision.has_follow() || decision.has_overtake() ||
               decision.has_yield()) {
      const auto ret = MapWithPredictionTrajectory(path_obstacle);
      if (!ret.ok()) {
        AERROR << "Fail to map obstacle " << path_obstacle->Id()
               << " with decision: " << decision.DebugString();
        return Status(ErrorCode::PLANNING_ERROR,
                      "Fail to map overtake/yield decision");
      }
    } else {
      ADEBUG << "No mapping for decision: " << decision.DebugString();
    }
  }

  if (stop_obstacle) {
    bool success = MapStopDecision(stop_obstacle);
    if (!success) {
      std::string msg = "Fail to MapStopDecision.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }
  return Status::OK();
}

bool StBoundaryMapper::MapStopDecision(PathObstacle* stop_obstacle) const {
  const auto& stop_decision = stop_obstacle->LongitudinalDecision();
  DCHECK(stop_decision.has_stop()) << "Must have stop decision";

  if (stop_obstacle->perception_sl_boundary().start_s() > planning_distance_) {
    return true;
  }

  double st_stop_s = 0.0;
  const double stop_ref_s = stop_obstacle->perception_sl_boundary().start_s() +
                            stop_decision.stop().distance_s() -
                            vehicle_param_.front_edge_to_center();

  if (stop_ref_s > path_data_.frenet_frame_path().points().back().s()) {
    st_stop_s =
        path_data_.discretized_path().EndPoint().s() +
        (stop_ref_s - path_data_.frenet_frame_path().points().back().s());
  } else {
    PathPoint stop_point;
    if (!path_data_.GetPathPointWithRefS(stop_ref_s, &stop_point)) {
      AERROR << "Fail to get path point from reference s. The sl boundary of "
                "stop obstacle "
             << stop_obstacle->Id() << " is: "
             << stop_obstacle->perception_sl_boundary().DebugString();
      return false;
    }

    st_stop_s = stop_point.s();
  }

  constexpr double kStopEpsilon = 1e-2;
  const double s_min = std::max(0.0, st_stop_s - kStopEpsilon);
  const double s_max =
      std::fmax(s_min, std::fmax(planning_distance_, reference_line_.Length()));

  std::vector<std::pair<STPoint, STPoint>> point_pairs;
  point_pairs.emplace_back(STPoint(s_min, 0.0), STPoint(s_max, 0.0));
  point_pairs.emplace_back(
      STPoint(s_min, planning_time_),
      STPoint(s_max + st_boundary_config_.boundary_buffer(), planning_time_));
  auto boundary = StBoundary(point_pairs);
  boundary.SetBoundaryType(StBoundary::BoundaryType::STOP);
  boundary.SetCharacteristicLength(st_boundary_config_.boundary_buffer());
  boundary.SetId(stop_obstacle->Id());
  stop_obstacle->SetStBoundary(boundary);
  return true;
}

Status StBoundaryMapper::MapWithoutDecision(PathObstacle* path_obstacle) const {
  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;

  if (!GetOverlapBoundaryPoints(path_data_.discretized_path().path_points(),
                                *(path_obstacle->obstacle()), &upper_points,
                                &lower_points)) {
    return Status::OK();
  }

  auto boundary = StBoundary::GenerateStBoundary(lower_points, upper_points)
                      .ExpandByS(boundary_s_buffer)
                      .ExpandByT(boundary_t_buffer);
  boundary.SetId(path_obstacle->Id());
  const auto& prev_st_boundary = path_obstacle->st_boundary();
  if (!prev_st_boundary.IsEmpty()) {
    boundary.SetBoundaryType(prev_st_boundary.boundary_type());
  }
  path_obstacle->SetStBoundary(boundary);
  return Status::OK();
}

bool StBoundaryMapper::GetOverlapBoundaryPoints(
    const std::vector<PathPoint>& path_points, const Obstacle& obstacle,
    std::vector<STPoint>* upper_points,
    std::vector<STPoint>* lower_points) const {
  DCHECK_NOTNULL(upper_points);
  DCHECK_NOTNULL(lower_points);
  DCHECK(upper_points->empty());
  DCHECK(lower_points->empty());
  DCHECK_GT(path_points.size(), 0);

  if (path_points.empty()) {
    AERROR << "No points in path_data_.discretized_path().";
    return false;
  }

  const auto& trajectory = obstacle.Trajectory();
  if (trajectory.trajectory_point_size() == 0) {
    if (!obstacle.IsStatic()) {
      AWARN << "Non-static obstacle[" << obstacle.Id()
            << "] has NO prediction trajectory."
            << obstacle.Perception().ShortDebugString();
    }
    for (const auto& curr_point_on_path : path_points) {
      if (curr_point_on_path.s() > planning_distance_) {
        break;
      }
      const Box2d obs_box = obstacle.PerceptionBoundingBox();

      if (CheckOverlap(curr_point_on_path, obs_box,
                       st_boundary_config_.boundary_buffer())) {
        lower_points->emplace_back(curr_point_on_path.s(), 0.0);
        lower_points->emplace_back(curr_point_on_path.s(), planning_time_);
        upper_points->emplace_back(planning_distance_, 0.0);
        upper_points->emplace_back(planning_distance_, planning_time_);
        break;
      }
    }
  } else {
    const int default_num_point = 50;
    DiscretizedPath discretized_path;
    if (path_points.size() > 2 * default_num_point) {
      const int ratio = path_points.size() / default_num_point;
      std::vector<PathPoint> sampled_path_points;
      for (size_t i = 0; i < path_points.size(); ++i) {
        if (i % ratio == 0) {
          sampled_path_points.push_back(path_points[i]);
        }
      }
      discretized_path.set_path_points(sampled_path_points);
    } else {
      discretized_path.set_path_points(path_points);
    }
    for (int i = 0; i < trajectory.trajectory_point_size(); ++i) {
      const auto& trajectory_point = trajectory.trajectory_point(i);
      if (i > 0) {
        const auto& pre_point = trajectory.trajectory_point(i - 1);
        if (trajectory_point.relative_time() <= pre_point.relative_time()) {
          AERROR << "Fail to map because prediction time is not increasing."
                 << "current point: " << trajectory_point.ShortDebugString()
                 << "previous point: " << pre_point.ShortDebugString();
          return false;
        }
      }

      const Box2d obs_box = obstacle.GetBoundingBox(trajectory_point);

      double trajectory_point_time = trajectory_point.relative_time();
      const double kNegtiveTimeThreshold = -1.0;
      if (trajectory_point_time < kNegtiveTimeThreshold) {
        continue;
      }

      const double step_length = vehicle_param_.front_edge_to_center();
      for (double path_s = 0.0; path_s < discretized_path.Length();
           path_s += step_length) {
        const auto curr_adc_path_point =
            discretized_path.EvaluateUsingLinearApproximation(
                path_s + discretized_path.StartPoint().s());
        if (CheckOverlap(curr_adc_path_point, obs_box,
                         st_boundary_config_.boundary_buffer())) {
          // found overlap, start searching with higher resolution
          const double backward_distance = -step_length;
          const double forward_distance = vehicle_param_.length() +
                                          vehicle_param_.width() +
                                          obs_box.length() + obs_box.width();
          const double default_min_step = 0.1;  // in meters
          const double fine_tuning_step_length = std::fmin(
              default_min_step, discretized_path.Length() / default_num_point);

          bool find_low = false;
          bool find_high = false;
          double low_s = std::fmax(0.0, path_s + backward_distance);
          double high_s =
              std::fmin(discretized_path.Length(), path_s + forward_distance);

          while (low_s < high_s) {
            if (find_low && find_high) {
              break;
            }
            if (!find_low) {
              const auto& point_low =
                  discretized_path.EvaluateUsingLinearApproximation(
                      low_s + discretized_path.StartPoint().s());
              if (!CheckOverlap(point_low, obs_box,
                                st_boundary_config_.boundary_buffer())) {
                low_s += fine_tuning_step_length;
              } else {
                find_low = true;
              }
            }
            if (!find_high) {
              const auto& point_high =
                  discretized_path.EvaluateUsingLinearApproximation(
                      high_s + discretized_path.StartPoint().s());
              if (!CheckOverlap(point_high, obs_box,
                                st_boundary_config_.boundary_buffer())) {
                high_s -= fine_tuning_step_length;
              } else {
                find_high = true;
              }
            }
          }
          if (find_high && find_low) {
            lower_points->emplace_back(
                low_s - st_boundary_config_.point_extension(),
                trajectory_point_time);
            upper_points->emplace_back(
                high_s + st_boundary_config_.point_extension(),
                trajectory_point_time);
          }
          break;
        }
      }
    }
  }
  DCHECK_EQ(lower_points->size(), upper_points->size());
  return (lower_points->size() > 1 && upper_points->size() > 1);
}

Status StBoundaryMapper::MapWithPredictionTrajectory(
    PathObstacle* path_obstacle) const {
  const auto& obj_decision = path_obstacle->LongitudinalDecision();
  DCHECK(obj_decision.has_follow() || obj_decision.has_yield() ||
         obj_decision.has_overtake())
      << "obj_decision must be follow or yield or overtake.\n"
      << obj_decision.DebugString();

  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;

  if (!GetOverlapBoundaryPoints(path_data_.discretized_path().path_points(),
                                *(path_obstacle->obstacle()), &upper_points,
                                &lower_points)) {
    return Status::OK();
  }

  if (obj_decision.has_follow() && lower_points.back().t() < planning_time_) {
    const double diff_s = lower_points.back().s() - lower_points.front().s();
    const double diff_t = lower_points.back().t() - lower_points.front().t();
    double extend_lower_s =
        diff_s / diff_t * (planning_time_ - lower_points.front().t()) +
        lower_points.front().s();
    const double extend_upper_s =
        extend_lower_s + (upper_points.back().s() - lower_points.back().s()) +
        1.0;
    upper_points.emplace_back(extend_upper_s, planning_time_);
    lower_points.emplace_back(extend_lower_s, planning_time_);
  }

  auto boundary = StBoundary::GenerateStBoundary(lower_points, upper_points)
                      .ExpandByS(boundary_s_buffer)
                      .ExpandByT(boundary_t_buffer);

  // get characteristic_length and boundary_type.
  StBoundary::BoundaryType b_type = StBoundary::BoundaryType::UNKNOWN;
  double characteristic_length = 0.0;
  if (obj_decision.has_follow()) {
    characteristic_length = std::fabs(obj_decision.follow().distance_s());
    b_type = StBoundary::BoundaryType::FOLLOW;
  } else if (obj_decision.has_yield()) {
    characteristic_length = std::fabs(obj_decision.yield().distance_s());
    boundary = StBoundary::GenerateStBoundary(lower_points, upper_points)
                   .ExpandByS(characteristic_length);
    b_type = StBoundary::BoundaryType::YIELD;
  } else if (obj_decision.has_overtake()) {
    characteristic_length = std::fabs(obj_decision.overtake().distance_s());
    b_type = StBoundary::BoundaryType::OVERTAKE;
  } else {
    DCHECK(false) << "Obj decision should be either yield or overtake: "
                  << obj_decision.DebugString();
  }
  boundary.SetBoundaryType(b_type);
  boundary.SetId(path_obstacle->obstacle()->Id());
  boundary.SetCharacteristicLength(characteristic_length);
  path_obstacle->SetStBoundary(boundary);
  return Status::OK();
}

bool StBoundaryMapper::CheckOverlap(const PathPoint& path_point,
                                    const Box2d& obs_box,
                                    const double buffer) const {
  Vec2d vec_to_center = Vec2d((vehicle_param_.front_edge_to_center() -
                               vehicle_param_.back_edge_to_center()) /
                                  2.0,
                              (vehicle_param_.left_edge_to_center() -
                               vehicle_param_.right_edge_to_center()) /
                                  2.0)
                            .rotate(path_point.theta());
  Vec2d center = Vec2d(path_point.x(), path_point.y()) + vec_to_center;

  const Box2d adc_box =
      Box2d(center, path_point.theta(), vehicle_param_.length() + 2 * buffer,
            vehicle_param_.width() + 2 * buffer);
  return obs_box.HasOverlap(adc_box);
}

void StBoundaryMapper::GetAvgKappa(
    const std::vector<common::PathPoint>& path_points,
    std::vector<double>* kappa) const {
  CHECK_NOTNULL(kappa);
  const int kHalfNumPoints = st_boundary_config_.num_points_to_avg_kappa() / 2;
  CHECK_GT(kHalfNumPoints, 0);
  kappa->clear();
  kappa->resize(path_points.size());
  double sum = 0.0;
  int start = 0;
  int end = 0;
  while (end < static_cast<int>(path_points.size()) &&
         end - start < kHalfNumPoints + 1) {
    sum += path_points[end].kappa();
    ++end;
  }

  int iter = 0;
  while (iter < static_cast<int>(path_points.size())) {
    kappa->at(iter) = sum / (end - start);
    if (start < iter - kHalfNumPoints) {
      sum -= path_points[start].kappa();
      ++start;
    }
    if (end < static_cast<int>(path_points.size())) {
      sum += path_points[end].kappa();
      ++end;
    }
    ++iter;
  }
}

Status StBoundaryMapper::GetSpeedLimits(
    const IndexedList<std::string, PathObstacle>& path_obstacles,
    SpeedLimit* const speed_limit_data) const {
  CHECK_NOTNULL(speed_limit_data);

  std::vector<double> avg_kappa;
  GetAvgKappa(path_data_.discretized_path().path_points(), &avg_kappa);
  const auto& discretized_path_points =
      path_data_.discretized_path().path_points();
  const auto& frenet_path_points = path_data_.frenet_frame_path().points();
  for (uint32_t i = 0; i < discretized_path_points.size(); ++i) {
    const double path_s = discretized_path_points.at(i).s();
    const auto& frenet_point_s = frenet_path_points.at(i).s();
    if (frenet_point_s > reference_line_.Length()) {
      AWARN << "path length [" << frenet_point_s
            << "] is LARGER than reference_line_ length ["
            << reference_line_.Length() << "]. Please debug before proceeding.";
      break;
    }

    // speed limit from map
    double speed_limit_on_reference_line =
        reference_line_.GetSpeedLimitFromS(frenet_point_s);

    // speed limit from path curvature
    const double centripetal_acceleration_limit =
        std::sqrt(st_boundary_config_.high_speed_centric_acceleration_limit() /
                  std::fmax(std::fabs(avg_kappa[i]),
                            st_boundary_config_.minimal_kappa()));

    // speed limit from nudge obstacles
    double nudge_obstacle_speed_limit = std::numeric_limits<double>::max();
    for (const auto* const_path_obstacle : path_obstacles.Items()) {
      if (const_path_obstacle->obstacle()->IsVirtual()) {
        continue;
      }
      if (!const_path_obstacle->LateralDecision().has_nudge()) {
        continue;
      }
      if (path_s < const_path_obstacle->perception_sl_boundary().start_s() ||
          path_s > const_path_obstacle->perception_sl_boundary().end_s()) {
        continue;
      }
      constexpr double kRange = 1.0;  // meters
      const auto& nudge = const_path_obstacle->LateralDecision().nudge();
      bool is_close_on_left =
          (nudge.type() == ObjectNudge::LEFT_NUDGE) &&
          (const_path_obstacle->perception_sl_boundary().end_l() > -kRange);
      bool is_close_on_right =
          (nudge.type() == ObjectNudge::RIGHT_NUDGE) &&
          (const_path_obstacle->perception_sl_boundary().start_l() < kRange);
      if (is_close_on_left || is_close_on_right) {
        double nudge_speed_ratio = 1.0;
        if (const_path_obstacle->obstacle()->IsStatic()) {
          nudge_speed_ratio =
              st_boundary_config_.static_obs_nudge_speed_ratio();
        } else {
          nudge_speed_ratio =
              st_boundary_config_.dynamic_obs_nudge_speed_ratio();
        }
        nudge_obstacle_speed_limit =
            nudge_speed_ratio * speed_limit_on_reference_line;
        break;
      }
    }

    double curr_speed_limit = 0.0;
    if (FLAGS_enable_nudge_slowdown) {
      curr_speed_limit = std::fmax(
        st_boundary_config_.lowest_speed(),
        common::util::MinElement(std::vector<double>{
            centripetal_acceleration_limit, speed_limit_on_reference_line,
            nudge_obstacle_speed_limit}));
    } else {
      curr_speed_limit = std::fmax(
        st_boundary_config_.lowest_speed(),
        common::util::MinElement(std::vector<double>{
            centripetal_acceleration_limit, speed_limit_on_reference_line}));
    }

    speed_limit_data->AppendSpeedLimit(path_s, curr_speed_limit);
  }
  return Status::OK();
}

double StBoundaryMapper::GetCentricAccLimit(const double kappa) const {
  // this function uses a linear model with upper and lower bound to determine
  // centric acceleration limit

  // suppose acc = k1 * v + k2
  // consider acc = v ^ 2 * kappa
  // we determine acc by the two functions above, with uppper and lower speed
  // bounds
  const double v_high = st_boundary_config_.high_speed_threshold();
  const double v_low = st_boundary_config_.low_speed_threshold();

  const double h_v_acc =
      st_boundary_config_.high_speed_centric_acceleration_limit();
  const double l_v_acc =
      st_boundary_config_.low_speed_centric_acceleration_limit();

  if (std::fabs(v_high - v_low) < 1.0) {
    AERROR << "High speed and low speed threshold are too close to each other. "
              "Please check config file."
           << " Current high speed threshold = " << v_high
           << ", current low speed threshold = " << v_low;
    return h_v_acc;
  }
  const double kMinKappaEpsilon = 1e-9;
  if (kappa < kMinKappaEpsilon) {
    return h_v_acc;
  }

  const double k1 = (h_v_acc - l_v_acc) / (v_high - v_low);
  const double k2 = h_v_acc - v_high * k1;

  const double v = (k1 + std::sqrt(k1 * k1 + 4 * kappa * k2)) / (2 * kappa);
  ADEBUG << "v = " << v;

  if (v > v_high) {
    return h_v_acc;
  } else if (v < v_low) {
    return l_v_acc;
  } else {
    return v * k1 + k2;
  }
}

}  // namespace planning
}  // namespace apollo
