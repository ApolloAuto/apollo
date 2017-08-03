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
 * @file: st_boundary_mapper_impl.cc
 **/

#include "modules/planning/optimizer/st_graph/st_boundary_mapper_impl.h"

#include <algorithm>
#include <limits>
#include <string>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/decision.pb.h"

#include "modules/common/log.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/vec2d.h"
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
using SLPoint = apollo::common::SLPoint;
using VehicleParam = apollo::common::VehicleParam;
using Box2d = apollo::common::math::Box2d;
using Vec2d = apollo::common::math::Vec2d;
using VehicleConfigHelper = apollo::common::VehicleConfigHelper;

Status StBoundaryMapperImpl::GetGraphBoundary(
    const common::TrajectoryPoint& initial_planning_point,
    const DecisionData& decision_data, const PathData& path_data,
    const ReferenceLine& reference_line, const double planning_distance,
    const double planning_time,
    std::vector<StGraphBoundary>* const st_graph_boundaries) const {
  if (st_graph_boundaries == nullptr) {
    const std::string msg = "st_graph_boundaries is NULL.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (planning_time < 0.0) {
    const std::string msg = "Fail to get params since planning_time < 0.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (path_data.discretized_path().num_of_points() < 2) {
    AERROR << "Fail to get params because of too few path points. path points "
              "size: "
           << path_data.discretized_path().num_of_points() << ".";
    return Status(ErrorCode::PLANNING_ERROR,
                  "Fail to get params because of too few path points");
  }

  st_graph_boundaries->clear();
  Status ret = Status::OK();

  const auto& main_decision = decision_data.Decision().main_decision();
  if (main_decision.has_stop()) {
    ret = MapMainDecisionStop(main_decision.stop(), reference_line,
                              planning_distance, planning_time,
                              st_graph_boundaries);
    if (!ret.ok() && ret.code() != ErrorCode::PLANNING_SKIP) {
      return Status(ErrorCode::PLANNING_ERROR);
    }
  } else if (main_decision.has_mission_complete()) {
    ret = MapMissionComplete(reference_line, planning_distance, planning_time,
                             st_graph_boundaries);
    if (!ret.ok() && ret.code() != ErrorCode::PLANNING_SKIP) {
      return Status(ErrorCode::PLANNING_ERROR);
    }
  }

  // TODO: enable mapping for static obstacles.
  /*
  const auto& static_obs_vec = decision_data.StaticObstacles();
  for (const auto* obs : static_obs_vec) {
    if (obs == nullptr) {
      continue;
    }
    ret = MapObstacleWithoutPredictionTrajectory(
        initial_planning_point, *obs, path_data, planning_distance,
        planning_time, st_graph_boundaries);
    if (!ret.ok()) {
      AERROR << "Fail to map static obstacle with id[" << obs->Id() << "].";
      return Status(ErrorCode::PLANNING_ERROR, "Fail to map static obstacle");
    }
  }
  */

  const auto& dynamic_obs_vec = decision_data.DynamicObstacles();
  for (const auto* obs : dynamic_obs_vec) {
    if (obs == nullptr) {
      continue;
    }
    for (auto& obj_decision : obs->Decisions()) {
      if (obj_decision.has_follow()) {
        ret = MapObstacleWithoutPredictionTrajectory(
            initial_planning_point, *obs, obj_decision, path_data,
            reference_line, planning_distance, planning_time,
            st_graph_boundaries);
        if (!ret.ok()) {
          AERROR << "Fail to map follow dynamic obstacle with id " << obs->Id()
                 << ".";
          return Status(ErrorCode::PLANNING_ERROR,
                        "Fail to map follow dynamic obstacle");
        }
      } else if (obj_decision.has_overtake() || obj_decision.has_yield()) {
        ret = MapObstacleWithPredictionTrajectory(
            initial_planning_point, *obs, obj_decision, path_data,
            planning_distance, planning_time, st_graph_boundaries);
        if (!ret.ok()) {
          AERROR << "Fail to map dynamic obstacle with id " << obs->Id() << ".";
          // Return OK by intention.
          return Status::OK();
        }
      }
    }
  }
  return Status::OK();
}

Status StBoundaryMapperImpl::MapMainDecisionStop(
    const MainStop& main_stop, const ReferenceLine& reference_line,
    const double planning_distance, const double planning_time,
    std::vector<StGraphBoundary>* const boundary) const {
  const auto lane_id =
      common::util::MakeMapId(main_stop.enforced_line().lane_id());
  const auto lane_info = Frame::PncMap()->HDMap()->get_lane_by_id(lane_id);
  const auto& map_point =
      lane_info->get_smooth_point(main_stop.enforced_line().distance_s());
  SLPoint sl_point;
  if (!reference_line.get_point_in_frenet_frame(
          Vec2d(map_point.x(), map_point.y()), &sl_point)) {
    AERROR << "Fail to MapMainDecisionStop since get_point_in_frenet_frame "
              "failed.";
    return Status(ErrorCode::PLANNING_ERROR);
  }
  sl_point.set_s(sl_point.s() - FLAGS_backward_routing_distance);
  const double stop_rear_center_s =
      sl_point.s() - FLAGS_decision_valid_stop_range -
      VehicleConfigHelper::GetConfig().vehicle_param().front_edge_to_center();
  if (Double::compare(stop_rear_center_s, 0.0) < 0) {
    AERROR << common::util::StrCat(
        "Fail to map main_decision_stop since stop_rear_center_s[",
        stop_rear_center_s, "] behind adc.");
  } else {
    if (stop_rear_center_s >=
        reference_line.length() - FLAGS_backward_routing_distance) {
      AWARN << common::util::StrCat(
          "Skip to MapMainDecisionStop since stop_rear_center_s[",
          stop_rear_center_s, "] > path length[", reference_line.length(),
          "].");
      return Status(ErrorCode::PLANNING_SKIP);
    }
  }
  const double s_min = (stop_rear_center_s > 0.0 ? stop_rear_center_s : 0.0);
  const double s_max = std::fmax(
      s_min + 1.0, std::fmax(planning_distance, reference_line.length()));

  std::vector<STPoint> boundary_points;
  boundary_points.emplace_back(s_min, 0.0);
  boundary_points.emplace_back(s_min, planning_time);
  boundary_points.emplace_back(s_max + st_boundary_config().boundary_buffer(),
                               planning_time);
  boundary_points.emplace_back(s_max, 0.0);

  const double area = GetArea(boundary_points);
  if (Double::compare(area, 0.0) <= 0) {
    return Status(ErrorCode::PLANNING_SKIP);
  }
  boundary->emplace_back(boundary_points);
  boundary->back().set_characteristic_length(
      st_boundary_config().boundary_buffer());
  boundary->back().set_boundary_type(StGraphBoundary::BoundaryType::STOP);
  return Status::OK();
}

Status StBoundaryMapperImpl::MapMissionComplete(
    const ReferenceLine& reference_line, const double planning_distance,
    const double planning_time,
    std::vector<StGraphBoundary>* const boundary) const {
  const double s_min = st_boundary_config().success_tunnel();
  const double s_max =
      std::fmin(planning_distance,
                reference_line.length() - FLAGS_backward_routing_distance);

  std::vector<STPoint> boundary_points;
  boundary_points.emplace_back(s_min, 0.0);
  boundary_points.emplace_back(s_max, 0.0);
  boundary_points.emplace_back(s_max + st_boundary_config().boundary_buffer(),
                               planning_time);
  boundary_points.emplace_back(s_min, planning_time);

  const double area = GetArea(boundary_points);
  if (Double::compare(area, 0.0) <= 0) {
    return Status(ErrorCode::PLANNING_SKIP);
  }
  boundary->emplace_back(boundary_points);
  boundary->back().set_characteristic_length(
      st_boundary_config().boundary_buffer());
  boundary->back().set_boundary_type(StGraphBoundary::BoundaryType::STOP);
  return Status::OK();
}

Status StBoundaryMapperImpl::MapObstacleWithPredictionTrajectory(
    const common::TrajectoryPoint& initial_planning_point,
    const Obstacle& obstacle, const ObjectDecisionType obj_decision,
    const PathData& path_data, const double planning_distance,
    const double planning_time,
    std::vector<StGraphBoundary>* const boundary) const {
  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;

  const double speed = obstacle.Speed();
  const double minimal_follow_time = st_boundary_config().minimal_follow_time();
  double follow_distance = -1.0;
  if (obj_decision.has_follow()) {
    follow_distance = std::fmax(speed * minimal_follow_time,
                                std::fabs(obj_decision.follow().distance_s())) +
                      vehicle_param().front_edge_to_center();
  }

  bool skip = true;
  std::vector<STPoint> boundary_points;
  const auto& adc_path_points = path_data.discretized_path().points();
  if (obstacle.prediction_trajectories().size() == 0) {
    AWARN << "Obstacle (id = " << obstacle.Id()
          << ") has NO prediction trajectory.";
  }

  for (uint32_t i = 0; i < obstacle.prediction_trajectories().size(); ++i) {
    const auto& trajectory = obstacle.prediction_trajectories()[i];
    for (int j = 0; j < trajectory.trajectory_point_size(); ++j) {
      const auto& trajectory_point = trajectory.trajectory_point(j);
      // TODO(all): fix trajectory point relative time issue.
      double trajectory_point_time = trajectory_point.relative_time();
      const Box2d obs_box(
          Vec2d(trajectory_point.path_point().x(),
                trajectory_point.path_point().y()),
          trajectory_point.path_point().theta(),
          obstacle.Length() * st_boundary_config().expending_coeff(),
          obstacle.Width() * st_boundary_config().expending_coeff());
      int64_t low = 0;
      int64_t high = adc_path_points.size() - 1;
      bool find_low = false;
      bool find_high = false;
      while (low < high) {
        if (find_low && find_high) {
          break;
        }
        if (!find_low) {
          if (!CheckOverlap(adc_path_points[low], vehicle_param(), obs_box,
                            st_boundary_config().boundary_buffer())) {
            ++low;
          } else {
            find_low = true;
          }
        }
        if (!find_high) {
          if (!CheckOverlap(adc_path_points[high], vehicle_param(), obs_box,
                            st_boundary_config().boundary_buffer())) {
            --high;
          } else {
            find_high = true;
          }
        }
      }
      if (find_high && find_low) {
        lower_points.emplace_back(
            adc_path_points[low].s() - st_boundary_config().point_extension(),
            trajectory_point_time);
        upper_points.emplace_back(
            adc_path_points[high].s() + st_boundary_config().point_extension(),
            trajectory_point_time);
      } else {
        if (obj_decision.has_yield() || obj_decision.has_overtake()) {
          AINFO << "Point[" << j << "] cannot find low or high index.";
        }
      }

      if (lower_points.size() > 0) {
        boundary_points.clear();
        const double buffer = st_boundary_config().follow_buffer();
        boundary_points.emplace_back(lower_points.at(0).s() - buffer,
                                     lower_points.at(0).t());
        boundary_points.emplace_back(lower_points.back().s() - buffer,
                                     lower_points.back().t());
        boundary_points.emplace_back(upper_points.back().s() + buffer +
                                         st_boundary_config().boundary_buffer(),
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
          boundary->back().set_boundary_type(b_type);
          skip = false;
        }
      }
    }
  }
  return skip ? Status(ErrorCode::PLANNING_SKIP, "PLANNING_SKIP")
              : Status::OK();
}

Status StBoundaryMapperImpl::MapObstacleWithoutPredictionTrajectory(
    const common::TrajectoryPoint& initial_planning_point,
    const Obstacle& obstacle, const ObjectDecisionType obj_decision,
    const PathData& path_data, const ReferenceLine& reference_line,
    const double planning_distance, const double planning_time,
    std::vector<StGraphBoundary>* const boundary) const {
  if (!obj_decision.has_follow()) {
    std::string msg = common::util::StrCat(
        "Map obstacle without prediction trajectory is ONLY supported when the "
        "object decision is follow. The current object decision is: \n",
        obj_decision.DebugString());
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  const double speed = obstacle.Speed();
  const double heading = obstacle.Heading();
  const Vec2d center = obstacle.Center();
  const Box2d box(center, heading,
                  obstacle.Length() * st_boundary_config().expending_coeff(),
                  obstacle.Width() * st_boundary_config().expending_coeff());

  const PathPoint ref_point =
      reference_line.get_reference_point(center.x(), center.y());
  const double speed_coeff = std::cos(heading - ref_point.theta());
  if (speed_coeff < 0.0) {
    std::string msg = common::util::StrCat(
        "Obstacle is moving opposite to the reference line. Obstacle heading: ",
        heading, "; ref_point.theta(): ", ref_point.theta());
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  const auto& point = path_data.discretized_path().points()[0];
  const PathPoint curr_point =
      reference_line.get_reference_point(point.x(), point.y());
  const double distance_to_obstacle =
      ref_point.s() - curr_point.s() -
      VehicleConfigHelper::GetConfig().vehicle_param().front_edge_to_center() -
      st_boundary_config().follow_buffer();

  if (distance_to_obstacle > planning_distance) {
    std::string msg = "obstacle is out of range.";
    AINFO << msg;
    return Status(ErrorCode::PLANNING_SKIP, msg);
  }

  double follow_speed = 0.0;
  if (speed > st_boundary_config().follow_speed_threshold()) {
    follow_speed = st_boundary_config().follow_speed_threshold() * speed_coeff;
  } else {
    follow_speed = speed * speed_coeff *
                   st_boundary_config().follow_speed_damping_factor();
  }

  const double s_min_lower = distance_to_obstacle;
  const double s_min_upper =
      std::max(distance_to_obstacle + 1.0, planning_distance);
  const double s_max_upper =
      std::max(s_min_upper + planning_time * follow_speed, planning_distance);
  const double s_max_lower = s_min_lower + planning_time * follow_speed;

  std::vector<STPoint> boundary_points;
  boundary_points.emplace_back(s_min_lower, 0.0);
  boundary_points.emplace_back(s_max_lower, planning_time);
  boundary_points.emplace_back(s_max_upper, planning_time);
  boundary_points.emplace_back(s_min_upper, 0.0);

  const double area = GetArea(boundary_points);
  if (Double::compare(area, 0.0) <= 0) {
    std::string msg = "Do not need to map because area is zero.";
    AINFO << msg;
    return Status(ErrorCode::PLANNING_SKIP, msg);
  }
  boundary->emplace_back(boundary_points);

  const double characteristic_length =
      std::fmax(
          speed * speed_coeff * st_boundary_config().minimal_follow_time(),
          std::fabs(obj_decision.follow().distance_s())) +
      VehicleConfigHelper::GetConfig().vehicle_param().front_edge_to_center() +
      st_boundary_config().follow_buffer();

  boundary->back().set_characteristic_length(
      characteristic_length * st_boundary_config().follow_coeff());
  boundary->back().set_boundary_type(StGraphBoundary::BoundaryType::FOLLOW);
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
