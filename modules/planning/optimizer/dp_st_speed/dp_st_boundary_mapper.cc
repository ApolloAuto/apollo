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
*   @file: dp_st_boundary_mapper.cpp
**/

#include "optimizer/dp_st_speed_optimizer/dp_st_boundary_mapper.h"

#include <limits>

#include "common/houston_gflags.h"
#include "math/double.h"
#include "math/segment2d.h"
#include "math/vec2d.h"

namespace apollo {
namespace planning {

DPSTBoundaryMapper::DPSTBoundaryMapper(
    const STBoundaryConfig& st_boundary_config,
    const ::adu::common::config::VehicleParam& veh_param)
    : StBoundaryMapper(st_boundary_config, veh_param) {}

ErrorCode DPSTBoundaryMapper::get_graph_boundary(
    const DataCenter& data_center, const DecisionData& decision_data,
    const PathData& path_data, const double planning_distance,
    const double planning_time,
    std::vector<STGraphBoundary>* const obs_boundary) const {
  QUIT_IF(planning_time < 0.0, ErrorCode::PLANNING_ERROR_FAILED, Level::ERROR,
          "Fail to get params since planning_time < 0.");
  QUIT_IF(path_data.path().num_of_points() < 2,
          ErrorCode::PLANNING_ERROR_FAILED, Level::ERROR,
          "Fail to get params since path has %lu points.",
          path_data.path().num_of_points());
  obs_boundary->clear();

  // TODO: add mapping main decision and map obstacle here
  const std::vector<const Obstacle*>& static_obs_vec =
      decision_data.static_obstacle();
  const std::vector<const Obstacle*>& dynamic_obs_vec =
      decision_data.dynamic_obstacle();

  for (std::size_t i = 0; i < static_obs_vec.size(); ++i) {
    if (static_obs_vec[i] == nullptr) {
      continue;
    }
    QUIT_IF(map_obstacle_without_trajectory(
                *static_obs_vec[i], path_data, planning_distance, planning_time,
                obs_boundary) != ErrorCode::PLANNING_OK,
            ErrorCode::PLANNING_ERROR_FAILED, Level::ERROR,
            "Fail to map static obstacle with id %d", static_obs_vec[i]->id());
  }

  for (std::size_t i = 0; i < dynamic_obs_vec.size(); ++i) {
    if (dynamic_obs_vec[i] == nullptr) {
      continue;
    }
    QUIT_IF(map_obstacle_with_trajectory(
                *dynamic_obs_vec[i], path_data, planning_distance,
                planning_time, obs_boundary) != ErrorCode::PLANNING_OK,
            ErrorCode::PLANNING_ERROR_FAILED, Level::ERROR,
            "Fail to map dynamic obstacle with id %d",
            dynamic_obs_vec[i]->id());
  }

  return ErrorCode::PLANNING_OK;
  ;
}

ErrorCode DPSTBoundaryMapper::map_obstacle_with_trajectory(
    const Obstacle& obstacle, const PathData& path_data,
    const double planning_distance, const double planning_time,
    std::vector<STGraphBoundary>* const boundary) const {
  // lower and upper bound for st boundary
  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;
  const std::vector<PathPoint>& veh_path = path_data.path().path_points();
  for (std::size_t i = 0; i < obstacle.prediction_trajectories().size(); ++i) {
    PredictionTrajectory pred_traj = obstacle.prediction_trajectories()[i];
    bool skip = true;
    for (std::size_t j = 0; j < pred_traj.num_of_points(); ++j) {
      TrajectoryPoint cur_obs_point = pred_traj.trajectory_point_at(j);
      // construct bounding box
      ::adu::common::math::Box2d obs_box({cur_obs_point.x(), cur_obs_point.y()},
                                         cur_obs_point.theta(),
                                         obstacle.length(), obstacle.width());

      // loop path data to find the lower and upper bound
      const double buffer = st_boundary_config().boundary_buffer();

      if (veh_path.size() == 0) {
        return ErrorCode::PLANNING_OK;
      }

      std::size_t low_index = 0;
      std::size_t high_index = veh_path.size() - 1;
      bool find_low = false;
      bool find_high = false;

      while (high_index >= low_index) {
        if (find_high && find_low) {
          break;
        }

        if (!find_low) {
          if (!check_overlap(veh_path[low_index], vehicle_param(), obs_box,
                             buffer)) {
            ++low_index;
          } else {
            find_low = true;
          }
        }

        if (!find_high) {
          if (!check_overlap(veh_path[high_index], vehicle_param(), obs_box,
                             buffer)) {
            --high_index;
          } else {
            find_high = true;
          }
        }
      }

      if (find_high && find_low) {
        double s_upper = std::min(veh_path[high_index].s(), planning_distance);
        double s_lower = std::min(veh_path[low_index].s(), planning_distance);
        if (Double::compare(s_lower, s_upper) >= 0) {
          continue;
        } else {
          skip = false;
          lower_points.emplace_back(s_lower, cur_obs_point.relative_time());
          upper_points.emplace_back(s_upper, cur_obs_point.relative_time());
        }
      }
    }
    if (skip) {
      continue;
    }

    std::vector<STPoint> boundary_points = lower_points;

    boundary_points.insert(boundary_points.end(), upper_points.rbegin(),
                           upper_points.rend());
  }

  return ErrorCode::PLANNING_OK;
}

ErrorCode DPSTBoundaryMapper::map_obstacle_without_trajectory(
    const Obstacle& obstacle, const PathData& path_data,
    const double planning_distance, const double planning_time,
    std::vector<STGraphBoundary>* const boundary) const {
  // Static obstacle only have yield option
  const std::vector<PathPoint>& veh_path = path_data.path().path_points();

  ::adu::common::math::Box2d obs_box = obstacle.bounding_box();

  const double buffer = st_boundary_config().boundary_buffer();

  QUIT_IF(veh_path.size() == 0, ErrorCode::PLANNING_ERROR_NOT_FOUND,
          Level::ERROR, "[DP_ST_BOUNDARY_MAPPER] Vehicle path empty!");

  std::size_t low_index = 0;
  std::size_t high_index = veh_path.size() - 1;
  bool find_low = false;
  bool find_high = false;

  while (high_index >= low_index) {
    if (find_high && find_low) {
      break;
    }

    if (!find_low) {
      if (!check_overlap(veh_path[low_index], vehicle_param(), obs_box,
                         buffer)) {
        ++low_index;
      } else {
        find_low = true;
      }
    }

    if (!find_high) {
      if (!check_overlap(veh_path[high_index], vehicle_param(), obs_box,
                         buffer)) {
        --high_index;
      } else {
        find_high = true;
      }
    }
  }

  std::vector<STPoint> boundary_points;
  if (find_high && find_low) {
    double s_upper = std::min(veh_path[high_index].s(), planning_distance);
    double s_lower = std::min(veh_path[low_index].s(), planning_distance);

    if (Double::compare(s_lower, s_upper) >= 0) {
      return ErrorCode::PLANNING_OK;
    } else {
      boundary_points.emplace_back(s_lower, 0.0);
      boundary_points.emplace_back(s_lower, planning_time);
      boundary_points.emplace_back(s_upper, planning_time);
      boundary_points.emplace_back(s_upper, 0.0);
      boundary->emplace_back(boundary_points);
    }
  }
  return ErrorCode::PLANNING_OK;
}

}  // namespace planning
}  // namespace apollo
