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
 * @file: qp_spline_st_boundary_mapper.cc
 **/

#include "modules/planning/optimizer/qp_spline_st_speed/qp_spline_st_boundary_mapper.h"

#include <algorithm>
#include <limits>

#include "modules/planning/proto/decision.pb.h"

#include "modules/common/log.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/math/double.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::Status;
using apollo::common::config::VehicleParam;
using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;

QPSplineSTBoundaryMapper::QPSplineSTBoundaryMapper(
    const STBoundaryConfig& st_boundary_config, const VehicleParam& veh_param)
    : STBoundaryMapper(st_boundary_config, veh_param) {}

Status QPSplineSTBoundaryMapper::get_graph_boundary(
    const common::TrajectoryPoint& initial_planning_point,
    const DecisionData& decision_data, const PathData& path_data,
    const double planning_distance, const double planning_time,
    std::vector<STGraphBoundary>* const obs_boundary) const {
  if (obs_boundary) {
    const std::string msg = "obs_boundary is NULL.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR_FAILED, msg);
  }

  if (planning_time < 0.0) {
    const std::string msg = "Fail to get params since planning_time < 0.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR_FAILED, msg);
  }

  if (path_data.path().num_of_points() < 2) {
    AERROR << "Fail to get params because of too few path points. path points "
              "size: "
           << path_data.path().num_of_points() << ".";
    return Status(ErrorCode::PLANNING_ERROR_FAILED,
                  "Fail to get params because of too few path points");
  }

  obs_boundary->clear();

  Status ret;
  /*
  const auto& planning_data = processed_data.planning_data();
  const auto& main_decision = planning_data.main_decision();
  QUIT_IF(!planning_data.has_main_decision(), ErrorCode::PLANNING_SKIP,
          Level::X_ERROR, "Skip mapping because no decision. \n%s",
          planning_data.DebugString().c_str());

  ErrorCode ret = ErrorCode::PLANNING_OK;

  if (main_decision.has_stop()) {
    ret = map_main_decision_stop(
        main_decision.stop(), processed_data_proxy.target_reference_line_raw(),
        planning_distance, planning_time, obs_boundary);
    QUIT_IF(ret != ErrorCode::PLANNING_OK && ret != ErrorCode::PLANNING_SKIP,
            ret, Level::X_ERROR,
            "Failed to get_params since map_main_decision_stop error.");
    obs_boundary->back().set_id("main decision stop");
  } else if (main_decision.has_change_lane()) {
    if (main_decision.change_lane().has_default_lane_stop()) {
      ret = map_main_decision_stop(
          main_decision.change_lane().default_lane_stop(),
          processed_data_proxy.default_reference_line_raw(), planning_distance,
          planning_time, obs_boundary);
      QUIT_IF(ret != ErrorCode::PLANNING_OK && ret != ErrorCode::PLANNING_SKIP,
              ret, Level::X_ERROR,
              "Fail to get_params due to map default lane stop error.");
      obs_boundary->back().set_id("default_lane stop");
    }
    if (main_decision.change_lane().has_target_lane_stop()) {
      ret = map_main_decision_stop(
          main_decision.change_lane().target_lane_stop(),
          processed_data_proxy.target_reference_line_raw(), planning_distance,
          planning_time, obs_boundary);
      QUIT_IF(ret != ErrorCode::PLANNING_OK && ret != ErrorCode::PLANNING_SKIP,
              ret, Level::X_ERROR,
              "Fail to get_params due to map target lane stop error.");
      obs_boundary->back().set_id("target_lane stop");
    }
  } else if (main_decision.has_mission_complete()) {
    ret = map_main_decision_mission_complete(
        processed_data_proxy.target_reference_line_raw(), planning_distance,
        planning_time, obs_boundary);
    QUIT_IF(ret != ErrorCode::PLANNING_OK && ret != ErrorCode::PLANNING_SKIP,
            ret, Level::X_ERROR,
            "Failed to get_params since map_mission_complete error.");
  }
  */

  const auto& static_obs_vec = decision_data.StaticObstacles();
  for (const auto* obs : static_obs_vec) {
    if (obs == nullptr) {
      continue;
    }
    ret = map_obstacle_without_trajectory(
        initial_planning_point, *obs, path_data, planning_distance,
        planning_time, obs_boundary);
    if (!ret.ok()) {
      AERROR << "Fail to map static obstacle with id[" << obs->Id() << "].";
      return Status(ErrorCode::PLANNING_ERROR_FAILED,
                    "Fail to map static obstacle");
    }
  }

  const auto& dynamic_obs_vec = decision_data.DynamicObstacles();
  for (const auto* obs : dynamic_obs_vec) {
    if (obs == nullptr) {
      continue;
    }
    for (auto& obj_decision : obs->Decisions()) {
      if (obj_decision.has_follow()) {
        ret = map_obstacle_with_planning(initial_planning_point, *obs,
                                         path_data, planning_distance,
                                         planning_time, obs_boundary);
        if (!ret.ok()) {
          AERROR << "Fail to map follow dynamic obstacle with id " << obs->Id()
                 << ".";
          return Status(ErrorCode::PLANNING_ERROR_FAILED,
                        "Fail to map follow dynamic obstacle");
        }
      } else if (obj_decision.has_overtake() || obj_decision.has_yield()) {
        ret = map_obstacle_with_prediction_trajectory(
            initial_planning_point, *obs, obj_decision, path_data,
            planning_distance, planning_time, obs_boundary);
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

Status QPSplineSTBoundaryMapper::map_obstacle_with_planning(
    const common::TrajectoryPoint& initial_planning_point,
    const Obstacle& obstacle, const PathData& path_data,
    const double planning_distance, const double planning_time,
    std::vector<STGraphBoundary>* const boundary) const {
  return Status::OK();
}

Status QPSplineSTBoundaryMapper::map_obstacle_with_prediction_trajectory(
    const common::TrajectoryPoint& initial_planning_point,
    const Obstacle& obstacle, const ObjectDecisionType obj_decision,
    const PathData& path_data, const double planning_distance,
    const double planning_time,
    std::vector<STGraphBoundary>* const boundary) const {
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
  const auto& adc_path_points = path_data.path().path_points();
  if (obstacle.prediction_trajectories().size() == 0) {
    AWARN << "Obstacle (id = " << obstacle.Id()
          << ") has NO prediction trajectory.";
  }

  for (uint32_t i = 0; i < obstacle.prediction_trajectories().size(); ++i) {
    const auto& trajectory = obstacle.prediction_trajectories()[i];
    for (uint32_t j = 0; j < trajectory.num_of_points(); ++i) {
      const auto& trajectory_point = trajectory.trajectory_point_at(j);
      // TODO: fix trajectory point relative time issue.
      double trajectory_point_time =
          trajectory_point.relative_time() +
          trajectory.start_timestamp();  // -curr_timestamp.
      const Box2d obs_box(
          Vec2d(trajectory_point.path_point().x(),
                trajectory_point.path_point().y()),
          trajectory_point.path_point().theta(),
          obstacle.Length() * st_boundary_config().expending_coeff(),
          obstacle.Width() * st_boundary_config().expending_coeff());
      int64_t low = 0;
      int64_t high = static_cast<int64_t>(path_data.path().num_of_points()) - 1;
      bool find_low = false;
      bool find_high = false;
      while (low < high) {
        if (find_low && find_high) {
          break;
        }
        if (!find_low) {
          if (!check_overlap(adc_path_points[low], vehicle_param(), obs_box,
                             st_boundary_config().boundary_buffer())) {
            ++low;
          } else {
            find_low = true;
          }
        }
        if (!find_high) {
          if (!check_overlap(adc_path_points[high], vehicle_param(), obs_box,
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
        STGraphBoundary::BoundaryType b_type =
            STGraphBoundary::BoundaryType::UNKNOWN;
        if (obj_decision.has_follow()) {
          boundary_points.at(0).set_s(boundary_points.at(0).s() -
                                      follow_distance);
          boundary_points.at(1).set_s(boundary_points.at(1).s() -
                                      follow_distance);
          boundary_points.at(3).set_t(-1.0);
          b_type = STGraphBoundary::BoundaryType::FOLLOW;
        } else if (obj_decision.has_yield()) {
          const double dis = std::fabs(obj_decision.yield().distance_s());
          // TODO: remove the arbitrary numbers in this part.
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
          b_type = STGraphBoundary::BoundaryType::YIELD;
        } else if (obj_decision.has_overtake()) {
          const double dis = std::fabs(obj_decision.overtake().distance_s());
          boundary_points.at(2).set_s(boundary_points.at(2).s() + dis);
          boundary_points.at(3).set_s(boundary_points.at(3).s() + dis);
        }

        const double area = get_area(boundary_points);
        if (Double::compare(area, 0.0) > 0) {
          boundary->emplace_back(boundary_points);
          boundary->back().set_boundary_type(b_type);
          skip = false;
        }
      }
    }
  }
  return skip ?
      Status(ErrorCode::PLANNING_SKIP, "PLANNING_SKIP") : Status::OK();
}

Status QPSplineSTBoundaryMapper::map_obstacle_without_trajectory(
    const common::TrajectoryPoint& initial_planning_point,
    const Obstacle& obstacle, const PathData& path_data,
    const double planning_distance, const double planning_time,
    std::vector<STGraphBoundary>* const boundary) const {
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
