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

#include "modules/planning/optimizer/st_graph/st_boundary_mapper.h"

#include <algorithm>
#include <limits>
#include <string>

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

using ErrorCode = apollo::common::ErrorCode;
using Status = apollo::common::Status;
using PathPoint = apollo::common::PathPoint;
using TrajectoryPoint = apollo::common::TrajectoryPoint;
using SLPoint = apollo::common::SLPoint;
using VehicleParam = apollo::common::VehicleParam;
using Box2d = apollo::common::math::Box2d;
using Vec2d = apollo::common::math::Vec2d;

StBoundaryMapper::StBoundaryMapper(
    const StBoundaryConfig& config, const ReferenceLine& reference_line,
    const PathData& path_data,
    const common::TrajectoryPoint& initial_planning_point,
    const double planning_distance, const double planning_time)
    : st_boundary_config_(config),
      reference_line_(reference_line),
      path_data_(path_data),
      vehicle_param_(
          common::VehicleConfigHelper::instance()->GetConfig().vehicle_param()),
      initial_planning_point_(initial_planning_point),
      planning_distance_(planning_distance),
      planning_time_(planning_time) {
  const auto& path_start_point = path_data_.discretized_path().start_point();
  common::SLPoint sl_point;
  DCHECK(reference_line_.get_point_in_frenet_frame(
      {path_start_point.x(), path_start_point.y()}, &sl_point))
      << "Failed to get adc reference line s";
  adc_front_s_ = sl_point.s() + vehicle_param_.front_edge_to_center();
}

Status StBoundaryMapper::GetGraphBoundary(
    const PathDecision& path_decision,
    std::vector<StGraphBoundary>* const st_graph_boundaries) const {
  const auto& path_obstacles = path_decision.path_obstacles();
  if (st_graph_boundaries == nullptr) {
    const std::string msg = "st_graph_boundaries is NULL.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (planning_time_ < 0.0) {
    const std::string msg = "Fail to get params since planning_time_ < 0.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (path_data_.discretized_path().num_of_points() < 2) {
    AERROR << "Fail to get params because of too few path points. path points "
              "size: "
           << path_data_.discretized_path().num_of_points() << ".";
    return Status(ErrorCode::PLANNING_ERROR,
                  "Fail to get params because of too few path points");
  }

  st_graph_boundaries->clear();
  Status ret = Status::OK();

  const PathObstacle* stop_obstacle = nullptr;
  ObjectDecisionType stop_decision;
  double min_stop_s = std::numeric_limits<double>::max();

  for (const auto& path_obstacle : path_obstacles.Items()) {
    const auto& obstacle = *path_obstacle->Obstacle();
    for (const auto& decision : path_obstacle->Decisions()) {
      StGraphBoundary boundary;
      if (decision.has_follow()) {
        const auto ret = MapFollowDecision(obstacle, decision, &boundary);
        if (!ret.ok()) {
          AERROR << "Fail to map obstacle " << path_obstacle->Id()
                 << " with follow decision: " << decision.DebugString();
          return Status(ErrorCode::PLANNING_ERROR,
                        "Fail to map follow decision");
        }
      } else if (decision.has_stop()) {
        const double stop_s = path_obstacle->sl_boundary().start_s() +
                              decision.stop().distance_s();
        if (stop_s < adc_front_s_) {
          AERROR << "Invalid stop decision. not stop at ahead of current "
                    "position. stop_s : "
                 << stop_s << ", and current adc_s is; " << adc_front_s_;
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
      } else if (decision.has_overtake() || decision.has_yield()) {
        const auto ret = MapObstacleWithPredictionTrajectory(
            obstacle, decision, st_graph_boundaries);
        if (!ret.ok()) {
          AERROR << "Fail to map obstacle " << path_obstacle->Id()
                 << " with decision: " << decision.DebugString();
          return Status(ErrorCode::PLANNING_ERROR,
                        "Fail to map overtake/yield decision");
        }
      } else {
        std::string msg = common::util::StrCat("No mapping for decision: ",
                                               decision.DebugString());
        return Status(ErrorCode::PLANNING_SKIP, msg);
      }
      boundary.set_id(path_obstacle->Id());
    }
  }

  if (stop_obstacle) {
    StGraphBoundary stop_boundary;
    bool success = MapObstacleWithStopDecision(*stop_obstacle, stop_decision,
                                               &stop_boundary);
    if (!success) {
      std::string msg = "Fail to MapObstacleWithStopDecision.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    st_graph_boundaries->push_back(stop_boundary);
  }

  return Status::OK();
}

bool StBoundaryMapper::MapObstacleWithStopDecision(
    const PathObstacle& stop_obstacle, const ObjectDecisionType& stop_decision,
    StGraphBoundary* const boundary) const {
  CHECK_NOTNULL(boundary);
  DCHECK(stop_decision.has_stop()) << "Must have stop decision";
  const double st_stop_s = stop_decision.stop().distance_s() +
                           stop_obstacle.sl_boundary().start_s() -
                           FLAGS_decision_valid_stop_range - adc_front_s_;
  if (st_stop_s < 0.0) {
    AERROR << "obstacle st stop_s " << st_stop_s
           << " is less than adc_front_s: " << adc_front_s_;
    return false;
  }

  const double s_min = st_stop_s;
  //TODO: what is 1.0 here?
  const double s_max = std::fmax(
      s_min + 1.0, std::fmax(planning_distance_, reference_line_.length()));
  std::vector<STPoint> boundary_points;
  boundary_points.emplace_back(s_min, 0.0);
  boundary_points.emplace_back(s_min, planning_time_);
  boundary_points.emplace_back(s_max + st_boundary_config_.boundary_buffer(),
                               planning_time_);
  boundary_points.emplace_back(s_max, 0.0);

  *boundary = StGraphBoundary(boundary_points);
  boundary->SetBoundaryType(StGraphBoundary::BoundaryType::STOP);
  boundary->SetCharacteristicLength(st_boundary_config_.boundary_buffer());
  return true;
}

Status StBoundaryMapper::MapObstacleWithPredictionTrajectory(
    const Obstacle& obstacle, const ObjectDecisionType& obj_decision,
    std::vector<StGraphBoundary>* const boundary) const {
  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;

  const auto& speed = obstacle.Perception().velocity();
  const double scalar_speed = std::hypot(speed.x(), speed.y());
  const double minimal_follow_time = st_boundary_config_.minimal_follow_time();
  double follow_distance = -1.0;
  if (obj_decision.has_follow()) {
    follow_distance = std::fmax(scalar_speed * minimal_follow_time,
                                std::fabs(obj_decision.follow().distance_s())) +
                      vehicle_param_.front_edge_to_center();
  }

  bool skip = true;
  std::vector<STPoint> boundary_points;
  const auto& adc_path_points = path_data_.discretized_path().points();
  const auto& trajectory = obstacle.Trajectory();
  if (trajectory.trajectory_point_size() == 0) {
    AWARN << "Obstacle (id = " << obstacle.Id()
          << ") has NO prediction trajectory.";
  }

  for (int j = 0; j < trajectory.trajectory_point_size(); ++j) {
    const auto& trajectory_point = trajectory.trajectory_point(j);
    // TODO(all): fix trajectory point relative time issue.
    double trajectory_point_time = trajectory_point.relative_time();
    const Box2d obs_box = obstacle.GetBoundingBox(trajectory_point);
    int64_t low = 0;
    int64_t high = adc_path_points.size() - 1;
    bool find_low = false;
    bool find_high = false;
    while (low < high) {
      if (find_low && find_high) {
        break;
      }
      if (!find_low) {
        if (!CheckOverlap(adc_path_points[low], obs_box,
                          st_boundary_config_.boundary_buffer())) {
          ++low;
        } else {
          find_low = true;
        }
      }
      if (!find_high) {
        if (!CheckOverlap(adc_path_points[high], obs_box,
                          st_boundary_config_.boundary_buffer())) {
          --high;
        } else {
          find_high = true;
        }
      }
    }
    if (find_high && find_low) {
      lower_points.emplace_back(
          adc_path_points[low].s() - st_boundary_config_.point_extension(),
          trajectory_point_time);
      upper_points.emplace_back(
          adc_path_points[high].s() + st_boundary_config_.point_extension(),
          trajectory_point_time);
    } else {
      if (obj_decision.has_yield() || obj_decision.has_overtake()) {
        AINFO << "Point[" << j << "] cannot find low or high index.";
      }
    }

    if (lower_points.size() > 0) {
      boundary_points.clear();
      const double buffer = st_boundary_config_.follow_buffer();
      boundary_points.emplace_back(lower_points.at(0).s() - buffer,
                                   lower_points.at(0).t());
      boundary_points.emplace_back(lower_points.back().s() - buffer,
                                   lower_points.back().t());
      boundary_points.emplace_back(upper_points.back().s() + buffer +
                                       st_boundary_config_.boundary_buffer(),
                                   upper_points.back().t());
      boundary_points.emplace_back(upper_points.at(0).s() + buffer,
                                   upper_points.at(0).t());
      if (lower_points.at(0).t() > lower_points.back().t() ||
          upper_points.at(0).t() > upper_points.back().t()) {
        AWARN << "lower/upper points are reversed.";
      }

      // change boundary according to obj_decision.
      StGraphBoundary::BoundaryType b_type =
          StGraphBoundary::BoundaryType::UNKNOWN;
      if (obj_decision.has_follow()) {
        boundary_points.at(0).set_s(boundary_points.at(0).s() -
                                    follow_distance);
        boundary_points.at(1).set_s(boundary_points.at(1).s() -
                                    follow_distance);
        boundary_points.at(3).set_t(-1.0);
        b_type = StGraphBoundary::BoundaryType::FOLLOW;
      } else if (obj_decision.has_yield()) {
        const double dis = std::fabs(obj_decision.yield().distance_s());
        // TODO(all): remove the arbitrary numbers in this part.
        if (boundary_points.at(0).s() - dis < 0.0) {
          boundary_points.at(0).set_s(
              std::fmax(boundary_points.at(0).s() - 2.0, 0.0));
        } else {
          boundary_points.at(0).set_s(
              std::fmax(boundary_points.at(0).s() - dis, 0.0));
        }
        if (boundary_points.at(1).s() - dis < 0.0) {
          boundary_points.at(1).set_s(
              std::fmax(boundary_points.at(0).s() - 4.0, 0.0));
        } else {
          boundary_points.at(1).set_s(
              std::fmax(boundary_points.at(0).s() - dis, 0.0));
        }
        b_type = StGraphBoundary::BoundaryType::YIELD;
      } else if (obj_decision.has_overtake()) {
        const double dis = std::fabs(obj_decision.overtake().distance_s());
        boundary_points.at(2).set_s(boundary_points.at(2).s() + dis);
        boundary_points.at(3).set_s(boundary_points.at(3).s() + dis);
      }

      const double area = GetArea(boundary_points);
      if (Double::compare(area, 0.0) > 0) {
        boundary->emplace_back(boundary_points);
        boundary->back().SetBoundaryType(b_type);
        boundary->back().set_id(obstacle.Id());
        skip = false;
      }
    }
  }
  return skip ? Status(ErrorCode::PLANNING_SKIP, "PLANNING_SKIP")
              : Status::OK();
}

Status StBoundaryMapper::MapFollowDecision(
    const Obstacle& obstacle, const ObjectDecisionType& obj_decision,
    StGraphBoundary* const boundary) const {
  if (!obj_decision.has_follow()) {
    std::string msg = common::util::StrCat(
        "Map obstacle without prediction trajectory is ONLY supported when "
        "the "
        "object decision is follow. The current object decision is: \n",
        obj_decision.DebugString());
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  const auto& speed = obstacle.Perception().velocity();
  const double scalar_speed = std::hypot(speed.x(), speed.y());

  const auto& perception = obstacle.Perception();
  const PathPoint ref_point = reference_line_.get_reference_point(
      perception.position().x(), perception.position().y());
  const double speed_coeff = std::cos(perception.theta() - ref_point.theta());
  if (speed_coeff < 0.0) {
    AERROR << "Obstacle is moving opposite to the reference line. Obstacle: "
           << perception.DebugString();
    return common::Status(ErrorCode::PLANNING_ERROR,
                          "obstacle is moving opposite the reference line");
  }

  const auto& point = path_data_.discretized_path().start_point();
  const PathPoint curr_point =
      reference_line_.get_reference_point(point.x(), point.y());
  const double distance_to_obstacle = ref_point.s() - curr_point.s() -
                                      vehicle_param_.front_edge_to_center() -
                                      st_boundary_config_.follow_buffer();

  if (distance_to_obstacle > planning_distance_) {
    std::string msg = "obstacle is out of range.";
    AINFO << msg;
    return Status(ErrorCode::PLANNING_SKIP, msg);
  }

  double follow_speed = 0.0;
  if (scalar_speed > st_boundary_config_.follow_speed_threshold()) {
    follow_speed = st_boundary_config_.follow_speed_threshold() * speed_coeff;
  } else {
    follow_speed = scalar_speed * speed_coeff *
                   st_boundary_config_.follow_speed_damping_factor();
  }

  const double s_min_lower = distance_to_obstacle;
  const double s_min_upper =
      std::max(distance_to_obstacle + 1.0, planning_distance_);
  const double s_max_upper =
      std::max(s_min_upper + planning_time_ * follow_speed, planning_distance_);
  const double s_max_lower = s_min_lower + planning_time_ * follow_speed;

  std::vector<STPoint> boundary_points;
  boundary_points.emplace_back(s_min_lower, 0.0);
  boundary_points.emplace_back(s_max_lower, planning_time_);
  boundary_points.emplace_back(s_max_upper, planning_time_);
  boundary_points.emplace_back(s_min_upper, 0.0);

  const double area = GetArea(boundary_points);
  if (Double::compare(area, 0.0) <= 0) {
    std::string msg = "Do not need to map because area is zero.";
    AINFO << msg;
    return Status(ErrorCode::PLANNING_SKIP, msg);
  }
  *boundary = StGraphBoundary(boundary_points);

  const double characteristic_length =
      std::fmax(scalar_speed * speed_coeff *
                    st_boundary_config_.minimal_follow_time(),
                std::fabs(obj_decision.follow().distance_s())) +
      vehicle_param_.front_edge_to_center() +
      st_boundary_config_.follow_buffer();

  boundary->SetCharacteristicLength(characteristic_length *
                                    st_boundary_config_.follow_coeff());
  boundary->SetBoundaryType(StGraphBoundary::BoundaryType::FOLLOW);
  boundary->set_id(obstacle.Id());
  return Status::OK();
}

double StBoundaryMapper::GetArea(
    const std::vector<STPoint>& boundary_points) const {
  if (boundary_points.size() < 3) {
    return 0.0;
  }

  double area = 0.0;
  for (uint32_t i = 2; i < boundary_points.size(); ++i) {
    const double ds1 = boundary_points[i - 1].s() - boundary_points[0].s();
    const double dt1 = boundary_points[i - 1].t() - boundary_points[0].t();

    const double ds2 = boundary_points[i].s() - boundary_points[0].s();
    const double dt2 = boundary_points[i].t() - boundary_points[0].t();
    // cross product
    area += (ds1 * dt2 - ds2 * dt1);
  }
  return fabs(area * 0.5);
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
  const Box2d adc_box = Box2d(
      {x, y}, path_point.theta(), vehicle_param_.length() + 2 * buffer,
      vehicle_param_.width() + 2 * buffer);
  return obs_box.HasOverlap(adc_box);
}

Status StBoundaryMapper::GetSpeedLimits(
    SpeedLimit* const speed_limit_data) const {
  CHECK_NOTNULL(speed_limit_data);

  std::vector<double> speed_limits;
  for (const auto& path_point : path_data_.discretized_path().points()) {
    if (Double::compare(path_point.s(), reference_line_.length()) > 0) {
      std::string msg = common::util::StrCat(
          "path length [", path_data_.discretized_path().length(),
          "] is LARGER than reference_line_ length [", reference_line_.length(),
          "]. Please debug before proceeding.");
      AWARN << msg;
      break;
    }

    // speed limit from reference line
    double speed_limit_on_reference_line =
        std::fmin(st_boundary_config_.maximal_speed(),
                  reference_line_.GetSpeedLimitFromS(path_point.s()));

    // speed limit from path curvature
    double speed_limit_on_path = std::sqrt(
        st_boundary_config_.centric_acceleration_limit() /
        std::fmax(path_point.kappa(), st_boundary_config_.minimal_kappa()));

    const double curr_speed_limit = std::fmax(
        st_boundary_config_.lowest_speed(),
        std::fmin(speed_limit_on_path, speed_limit_on_reference_line));

    common::SpeedPoint speed_point;
    speed_point.set_s(path_point.s());
    speed_point.set_v(curr_speed_limit);
    speed_limit_data->add_speed_limit(speed_point);
  }
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
