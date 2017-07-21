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

using ErrorCode = apollo::common::ErrorCode;
using VehicleParam = apollo::common::config::VehicleParam;
using STPoint = apollo::common::STPoint;
using PathPoint = apollo::common::PathPoint;

QpSplineStBoundaryMapper::QpSplineStBoundaryMapper(
    const StBoundaryConfig& st_boundary_config, const VehicleParam& veh_param)
    : StBoundaryMapper(st_boundary_config, veh_param) {}

ErrorCode QpSplineStBoundaryMapper::get_graph_boundary(
    const DataCenter& data_center, const DecisionData& decision_data,
    const PathData& path_data, const double planning_distance,
    const double planning_time,
    std::vector<STGraphBoundary>* const obs_boundary) const {
  if (planning_time < 0.0) {
    AERROR << "Fail to get params because planning_time < 0.";
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  if (path_data.path().num_of_points() < 2) {
    AERROR << "Fail to get params since path has "
           << path_data.path().num_of_points() << "points.";
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  obs_boundary->clear();

  // TODO: add mapping main decision and map obstacle here
  const std::vector<const Obstacle*>& static_obs_vec =
      decision_data.StaticObstacles();
  const std::vector<const Obstacle*>& dynamic_obs_vec =
      decision_data.DynamicObstacles();

  for (std::size_t i = 0; i < static_obs_vec.size(); ++i) {
    if (static_obs_vec[i] == nullptr) {
      continue;
    }
    if (map_obstacle_without_trajectory(
            *static_obs_vec[i], path_data, planning_distance, planning_time,
            obs_boundary) != ErrorCode::PLANNING_OK) {
      AERROR << "Fail to map static obstacle with id "
             << static_obs_vec[i]->Id();
      return ErrorCode::PLANNING_ERROR_FAILED;
    }
  }

  for (std::size_t i = 0; i < dynamic_obs_vec.size(); ++i) {
    if (dynamic_obs_vec[i] == nullptr) {
      continue;
    }
    if (map_obstacle_with_trajectory(
            data_center, *dynamic_obs_vec[i], path_data, planning_distance,
            planning_time, obs_boundary) != ErrorCode::PLANNING_OK) {
      AERROR << "Fail to map dynamic obstacle with id "
             << dynamic_obs_vec[i]->Id();
      return ErrorCode::PLANNING_ERROR_FAILED;
    }
  }

  return ErrorCode::PLANNING_OK;
}

ErrorCode QpSplineStBoundaryMapper::map_obstacle_with_trajectory(
    const DataCenter& data_center, const Obstacle& obstacle,
    const PathData& path_data, const double planning_distance,
    const double planning_time,
    std::vector<STGraphBoundary>* const boundary) const {
  std::vector<STPoint> boundary_points;
  // lower and upper bound for st boundary
  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;
  const std::vector<PathPoint>& veh_path = path_data.path().path_points();
  const std::vector<Decision>& decision = obstacle.Decisions();
  for (std::size_t i = 0; i < obstacle.prediction_trajectories().size(); ++i) {
    PredictionTrajectory pred_traj = obstacle.prediction_trajectories()[i];
    bool skip = true;
    const double boundary_buffer = st_boundary_config().boundary_buffer();

    for (std::size_t j = 0; j < pred_traj.num_of_points(); ++j) {
      TrajectoryPoint cur_obs_point = pred_traj.trajectory_point_at(j);
      // construct bounding box
      apollo::common::math::Box2d obs_box(
          {cur_obs_point.path_point().x(), cur_obs_point.path_point().y()},
          cur_obs_point.path_point().theta(),
          obstacle.Length() * st_boundary_config().expending_coeff(),
          obstacle.Width() * st_boundary_config().expending_coeff());

      // loop path data to find the lower and upper bound
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
                             boundary_buffer)) {
            ++low_index;
          } else {
            find_low = true;
          }
        }

        if (!find_high) {
          if (!check_overlap(veh_path[high_index], vehicle_param(), obs_box,
                             boundary_buffer)) {
            --high_index;
          } else {
            find_high = true;
          }
        }
      }

      if (find_high && find_low) {
        double s_upper = std::min(
            veh_path[high_index].s() + st_boundary_config().point_extension(),
            planning_distance);
        double s_lower = std::min(
            veh_path[low_index].s() - st_boundary_config().point_extension(),
            planning_distance);
        if (Double::compare(s_lower, s_upper) >= 0) {
          continue;
        } else {
          skip = false;
          lower_points.push_back(common::util::MakeSTPoint(
              s_lower, cur_obs_point.relative_time()));
          upper_points.push_back(common::util::MakeSTPoint(
              s_upper, cur_obs_point.relative_time()));
        }
      }
    }
    if (skip) {
      continue;
    }
    if (lower_points.size() > 0) {
      boundary_points.clear();
      const double follow_buffer = st_boundary_config().follow_buffer();
      boundary_points.push_back(common::util::MakeSTPoint(
          lower_points.at(0).s() - follow_buffer, lower_points.at(0).t()));
      boundary_points.push_back(common::util::MakeSTPoint(
          lower_points.back().s() - follow_buffer, lower_points.back().t()));
      boundary_points.push_back(common::util::MakeSTPoint(
          upper_points.back().s() + follow_buffer + boundary_buffer,
          upper_points.back().t()));
      boundary_points.push_back(common::util::MakeSTPoint(
          upper_points.at(0).s() + follow_buffer, upper_points.at(0).t()));

      if (lower_points.at(0).t() > lower_points.back().t() ||
          upper_points.at(0).t() > upper_points.back().t()) {
        AINFO << "Warning: reverse t vs s. t: lower (first:"
              << lower_points.at(0).t() << ", back:" << lower_points.back().t()
              << "), upper (first:" << upper_points.at(0).t()
              << ", back:" << upper_points.back().t() << ")";
      }
      const double decision_buffer = decision[i].buffer();
      Decision::DecisionType decision_type;
      if (decision[i].decision_type() == Decision::DecisionType::YIELD_DOWN ||
          decision[i].decision_type() == Decision::DecisionType::STOP_DOWN ||
          decision[i].decision_type() == Decision::DecisionType::FOLLOW_DOWN) {
        if (boundary_points[0].s() - decision_buffer < 0.0) {
          boundary_points[0].set_s(
              std::fmax(boundary_points[0].s() - 2.0, 0.0));
        } else {
          boundary_points[0].set_s(
              std::fmax(boundary_points[0].s() - decision_buffer, 0.0));
        }
        if (boundary_points[1].s() - decision_buffer < 0.0) {
          boundary_points[1].set_s(
              std::fmax(boundary_points[1].s() - 4.0, 0.0));
        } else {
          boundary_points[1].set_s(
              std::fmax(boundary_points[1].s() - decision_buffer, 0.0));
        }
        decision_type = Decision::DecisionType::YIELD_DOWN;
      } else if (decision[i].decision_type() == Decision::DecisionType::GO_UP) {
        boundary_points[2].set_s(boundary_points[2].s() + decision_buffer);
        boundary_points[3].set_s(boundary_points[3].s() + decision_buffer);
        decision_type = Decision::DecisionType::GO_UP;
      }
      const double area = get_area(boundary_points);
      if (Double::compare(area, 0.0) > 0) {
        // main boundary
        boundary->emplace_back(boundary_points);
        boundary->back().set_decision_type(decision_type);

        // buffer boundary
        std::vector<STPoint> left_buffer_boundary_points;
        std::vector<STPoint> right_buffer_boundary_points;
        double compute_delay_time = 0.3;
        double control_cmd_delay_time = 0.3;
        double control_throttle_release_time = 0.1;
        double control_brake_full_time = 0.2;
        double t_delay = compute_delay_time + control_cmd_delay_time;
        const double init_acc = data_center.current_frame()
                                    ->planning_data()
                                    .init_planning_point()
                                    .a();
        const double init_speed = data_center.current_frame()
                                      ->planning_data()
                                      .init_planning_point()
                                      .v();
        if (init_acc > 0.3) {
          t_delay += control_throttle_release_time;
        }
        t_delay += control_brake_full_time;
        double s_delay = init_speed * t_delay;
        if (init_acc > 0.1) {
          s_delay += 0.5 * init_acc * t_delay * t_delay;
        }

        if (decision_type == Decision::DecisionType::YIELD_DOWN) {
          left_buffer_boundary_points.push_back(common::util::MakeSTPoint(
              std::fmax((boundary_points[0].s() - s_delay), 0.1),
              boundary_points[0].t() - 1.0));
          left_buffer_boundary_points.push_back(common::util::MakeSTPoint(
              std::fmax((boundary_points[0].s() - s_delay), 0.1) + 0.2,
              boundary_points[0].t()));
          left_buffer_boundary_points.push_back(common::util::MakeSTPoint(
              boundary_points[3].s(), boundary_points[3].t()));
          left_buffer_boundary_points.push_back(
              common::util::MakeSTPoint(left_buffer_boundary_points[0].s(),
                                        left_buffer_boundary_points[0].t()));

          right_buffer_boundary_points.push_back(common::util::MakeSTPoint(
              boundary_points[1].s(), boundary_points[1].t()));
          right_buffer_boundary_points.push_back(common::util::MakeSTPoint(
              boundary_points[1].s(), boundary_points[1].t() + 1.0));
          right_buffer_boundary_points.push_back(
              common::util::MakeSTPoint(right_buffer_boundary_points[1].s(),
                                        right_buffer_boundary_points[1].t()));
          right_buffer_boundary_points.push_back(common::util::MakeSTPoint(
              boundary_points[2].s(), boundary_points[2].t()));
        }
        if (decision_type == Decision::DecisionType::GO_UP) {
          left_buffer_boundary_points.push_back(common::util::MakeSTPoint(
              boundary_points[3].s(),
              std::max(boundary_points[3].t() - 0.5, 0.1)));
          left_buffer_boundary_points.push_back(common::util::MakeSTPoint(
              boundary_points[0].s(), boundary_points[0].t()));
          left_buffer_boundary_points.push_back(common::util::MakeSTPoint(
              boundary_points[3].s(), boundary_points[3].t()));
          left_buffer_boundary_points.push_back(
              common::util::MakeSTPoint(left_buffer_boundary_points[0].s(),
                                        left_buffer_boundary_points[0].t()));

          right_buffer_boundary_points.push_back(common::util::MakeSTPoint(
              boundary_points[1].s(), boundary_points[1].t()));
          right_buffer_boundary_points.push_back(common::util::MakeSTPoint(
              boundary_points[2].s() + 1.0, boundary_points[2].t() + 1.0));
          right_buffer_boundary_points.push_back(
              common::util::MakeSTPoint(right_buffer_boundary_points[1].s(),
                                        right_buffer_boundary_points[1].t()));
          right_buffer_boundary_points.push_back(common::util::MakeSTPoint(
              boundary_points[2].s(), boundary_points[2].t()));
        }

        boundary->emplace_back(left_buffer_boundary_points);
        boundary->back().set_decision_type(decision_type);
        boundary->emplace_back(right_buffer_boundary_points);
        boundary->back().set_decision_type(decision_type);
      } else {
        AWARN << "Can not map one of prediction trajectory to ST graph. ["
              << i + 1 << "/" << obstacle.prediction_trajectories().size()
              << "].";
      }
    } else {
      AWARN << "Did not find a overlap along the trajecotry of prediction. ["
            << i + 1 << "/" << obstacle.prediction_trajectories().size()
            << "].";
    }
  }

  return ErrorCode::PLANNING_OK;
}

ErrorCode QpSplineStBoundaryMapper::map_obstacle_without_trajectory(
    const Obstacle& obstacle, const PathData& path_data,
    const double planning_distance, const double planning_time,
    std::vector<STGraphBoundary>* const boundary) const {
  // Static obstacle only have yield option
  const std::vector<PathPoint>& veh_path = path_data.path().path_points();

  apollo::common::math::Box2d obs_box = obstacle.BoundingBox();

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

  std::vector<STPoint> boundary_points;
  if (find_high && find_low) {
    double s_upper = std::min(veh_path[high_index].s(), planning_distance);
    double s_lower = std::min(veh_path[low_index].s(), planning_distance);

    if (Double::compare(s_lower, s_upper) >= 0) {
      return ErrorCode::PLANNING_OK;
    } else {
      boundary_points.push_back(common::util::MakeSTPoint(s_lower, 0.0));
      boundary_points.push_back(
          common::util::MakeSTPoint(s_lower, planning_time));
      boundary_points.push_back(
          common::util::MakeSTPoint(s_upper, planning_time));
      boundary_points.push_back(common::util::MakeSTPoint(s_upper, 0.0));

      boundary->emplace_back(boundary_points);
      boundary->back().set_decision_type(Decision::DecisionType::YIELD_DOWN);
    }
  }
  return ErrorCode::PLANNING_OK;
}

}  // namespace planning
}  // namespace apollo
