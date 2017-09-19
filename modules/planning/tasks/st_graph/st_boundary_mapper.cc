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
#include <string>
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
#include "modules/common/vehicle_state/vehicle_state.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/math/double.h"

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

StBoundaryMapper::StBoundaryMapper(const hdmap::PncMap* pnc_map,
                                   const SLBoundary& adc_sl_boundary,
                                   const StBoundaryConfig& config,
                                   const ReferenceLine& reference_line,
                                   const PathData& path_data,
                                   const double planning_distance,
                                   const double planning_time)
    : pnc_map_(pnc_map),
      adc_sl_boundary_(adc_sl_boundary),
      st_boundary_config_(config),
      reference_line_(reference_line),
      path_data_(path_data),
      vehicle_param_(
          common::VehicleConfigHelper::instance()->GetConfig().vehicle_param()),
      planning_distance_(planning_distance),
      planning_time_(planning_time) {}

Status StBoundaryMapper::GetGraphBoundary(
    const PathDecision& path_decision,
    std::vector<StBoundary>* const st_boundaries) const {
  const auto& path_obstacles = path_decision.path_obstacles();
  if (st_boundaries == nullptr) {
    const std::string msg = "st_boundaries is NULL.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

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

  st_boundaries->clear();

  const PathObstacle* stop_obstacle = nullptr;
  ObjectDecisionType stop_decision;
  double min_stop_s = std::numeric_limits<double>::max();

  for (const auto* path_obstacle : path_obstacles.Items()) {
    if (!path_obstacle->HasLongitudinalDecision()) {
      StBoundary boundary;
      const auto ret = MapWithoutDecision(*path_obstacle, &boundary);
      if (!ret.ok()) {
        std::string msg = common::util::StrCat(
            "Fail to map obstacle ", path_obstacle->Id(), " without decision.");
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
      }
      AppendBoundary(boundary, st_boundaries);
      continue;
    }
    const auto& decision = path_obstacle->LongitudinalDecision();
    if (decision.has_stop()) {
      const double stop_s = path_obstacle->perception_sl_boundary().start_s() +
                            decision.stop().distance_s();
      if (stop_s < adc_sl_boundary_.end_s()) {
        AERROR << "Invalid stop decision. not stop at ahead of current "
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
      StBoundary boundary;
      const auto ret =
          MapWithPredictionTrajectory(*path_obstacle, decision, &boundary);
      if (!ret.ok()) {
        AERROR << "Fail to map obstacle " << path_obstacle->Id()
               << " with decision: " << decision.DebugString();
        return Status(ErrorCode::PLANNING_ERROR,
                      "Fail to map overtake/yield decision");
      }
      AppendBoundary(boundary, st_boundaries);
    } else {
      ADEBUG << "No mapping for decision: " << decision.DebugString();
    }
  }

  if (stop_obstacle) {
    StBoundary stop_boundary;
    bool success =
        MapStopDecision(*stop_obstacle, stop_decision, &stop_boundary);
    if (!success) {
      std::string msg = "Fail to MapStopDecision.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    AppendBoundary(stop_boundary, st_boundaries);
  }
  for (const auto& st_boundary : *st_boundaries) {
    DCHECK_NE(st_boundary.id().length(), 0);
  }
  return Status::OK();
}

bool StBoundaryMapper::MapStopDecision(const PathObstacle& stop_obstacle,
                                       const ObjectDecisionType& stop_decision,
                                       StBoundary* const boundary) const {
  CHECK_NOTNULL(boundary);
  DCHECK(stop_decision.has_stop()) << "Must have stop decision";

  PathPoint obstacle_point;
  if (stop_obstacle.perception_sl_boundary().start_s() >
      path_data_.frenet_frame_path().points().back().s()) {
    return true;
  }
  if (!path_data_.GetPathPointWithRefS(
          stop_obstacle.perception_sl_boundary().start_s(), &obstacle_point)) {
    AERROR << "Fail to get path point from reference s. The sl boundary of "
              "stop obstacle "
           << stop_obstacle.Id()
           << " is: " << stop_obstacle.perception_sl_boundary().DebugString();
    return false;
  }

  const double st_stop_s =
      obstacle_point.s() + stop_decision.stop().distance_s() -
      vehicle_param_.front_edge_to_center() - FLAGS_decision_valid_stop_range;
  if (st_stop_s < 0.0) {
    AERROR << "obstacle " << stop_obstacle.Id() << " st stop_s " << st_stop_s
           << " is less than 0.";
    return false;
  }

  const double s_min = st_stop_s;
  const double s_max =
      std::fmax(s_min, std::fmax(planning_distance_, reference_line_.Length()));

  std::vector<std::pair<STPoint, STPoint>> point_pairs;
  point_pairs.emplace_back(STPoint(s_min, 0.0), STPoint(s_max, 0.0));
  point_pairs.emplace_back(
      STPoint(s_min, planning_time_),
      STPoint(s_max + st_boundary_config_.boundary_buffer(), planning_time_));
  *boundary = StBoundary(point_pairs);

  boundary->SetBoundaryType(StBoundary::BoundaryType::STOP);
  boundary->SetCharacteristicLength(st_boundary_config_.boundary_buffer());
  boundary->SetId(stop_obstacle.Id());
  return true;
}

Status StBoundaryMapper::MapWithoutDecision(const PathObstacle& path_obstacle,
                                            StBoundary* const boundary) const {
  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;

  if (!GetOverlapBoundaryPoints(path_data_.discretized_path().path_points(),
                                *(path_obstacle.obstacle()), &upper_points,
                                &lower_points)) {
    return Status::OK();
  }

  if (lower_points.size() != upper_points.size()) {
    std::string msg = common::util::StrCat(
        "lower_points.size()[", lower_points.size(),
        "] and upper_points.size()[", upper_points.size(), "] does NOT match.");
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (lower_points.size() > 1 && upper_points.size() > 1) {
    *boundary = StBoundary::GenerateStBoundary(lower_points, upper_points)
                    .ExpandByS(boundary_s_buffer)
                    .ExpandByT(boundary_t_buffer);
    boundary->SetId(path_obstacle.obstacle()->Id());
  }
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
  return (lower_points->size() > 0 && upper_points->size() > 0);
}

Status StBoundaryMapper::MapWithPredictionTrajectory(
    const PathObstacle& path_obstacle, const ObjectDecisionType& obj_decision,
    StBoundary* const boundary) const {
  DCHECK_NOTNULL(boundary);
  DCHECK(obj_decision.has_follow() || obj_decision.has_yield() ||
         obj_decision.has_overtake())
      << "obj_decision must be follow or yield or overtake.\n"
      << obj_decision.DebugString();

  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;

  if (!GetOverlapBoundaryPoints(path_data_.discretized_path().path_points(),
                                *(path_obstacle.obstacle()), &upper_points,
                                &lower_points)) {
    return Status(ErrorCode::PLANNING_ERROR, "PLANNING_ERROR");
  }

  if (lower_points.size() != upper_points.size()) {
    std::string msg = common::util::StrCat(
        "lower_points.size()[", lower_points.size(),
        "] and upper_points.size()[", upper_points.size(), "] does NOT match.");
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (lower_points.size() > 1 && upper_points.size() > 1) {
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

    *boundary = StBoundary::GenerateStBoundary(lower_points, upper_points)
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
      b_type = StBoundary::BoundaryType::YIELD;
    } else if (obj_decision.has_overtake()) {
      characteristic_length = std::fabs(obj_decision.overtake().distance_s());
      b_type = StBoundary::BoundaryType::OVERTAKE;
    } else {
      DCHECK(false) << "Obj decision should be either yield or overtake: "
                    << obj_decision.DebugString();
    }
    boundary->SetBoundaryType(b_type);
    boundary->SetId(path_obstacle.obstacle()->Id());
    boundary->SetCharacteristicLength(characteristic_length);
  }
  return Status::OK();
}

bool StBoundaryMapper::CheckOverlap(const PathPoint& path_point,
                                    const Box2d& obs_box,
                                    const double buffer) const {
  const double mid_to_rear_center =
      vehicle_param_.length() / 2.0 - vehicle_param_.front_edge_to_center();
  const double x =
      path_point.x() - mid_to_rear_center * std::cos(path_point.theta());
  const double y =
      path_point.y() - mid_to_rear_center * std::sin(path_point.theta());
  const Box2d adc_box =
      Box2d({x, y}, path_point.theta(), vehicle_param_.length() + 2 * buffer,
            vehicle_param_.width() + 2 * buffer);
  return obs_box.HasOverlap(adc_box);
}

Status StBoundaryMapper::GetSpeedLimits(
    SpeedLimit* const speed_limit_data) const {
  CHECK_NOTNULL(speed_limit_data);

  for (const auto& path_point : path_data_.discretized_path().path_points()) {
    if (Double::Compare(path_point.s(), reference_line_.Length()) > 0) {
      AWARN << "path length [" << path_data_.discretized_path().Length()
            << "] is LARGER than reference_line_ length ["
            << reference_line_.Length() << "]. Please debug before proceeding.";
      break;
    }

    double speed_limit_on_reference_line =
        reference_line_.GetSpeedLimitFromS(path_point.s());

    // speed limit from path curvature
    double speed_limit_on_path =
        std::sqrt(st_boundary_config_.centric_acceleration_limit() /
                  std::fmax(std::fabs(path_point.kappa()),
                            st_boundary_config_.minimal_kappa()));

    const double curr_speed_limit = std::fmax(
        st_boundary_config_.lowest_speed(),
        std::fmin(speed_limit_on_path, speed_limit_on_reference_line));

    speed_limit_data->AppendSpeedLimit(path_point.s(), curr_speed_limit);
  }
  return Status::OK();
}

void StBoundaryMapper::AppendBoundary(
    const StBoundary& boundary, std::vector<StBoundary>* st_boundaries) const {
  if (Double::Compare(boundary.Area(), 0.0) <= 0) {
    return;
  }
  st_boundaries->push_back(std::move(boundary));
}

}  // namespace planning
}  // namespace apollo
