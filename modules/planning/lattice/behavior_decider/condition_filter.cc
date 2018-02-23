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

#include "modules/planning/lattice/behavior_decider/condition_filter.h"

#include <algorithm>
#include <cmath>
#include <string>

#include "modules/common/log.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::math::lerp;

ConditionFilter::ConditionFilter(
    const std::array<double, 3>& init_s, const double speed_limit,
    std::shared_ptr<PathTimeGraph> ptr_path_time_graph,
    std::shared_ptr<PredictionQuerier> ptr_prediction_obstacles)
    : init_s_(init_s),
      feasible_region_(init_s, speed_limit),
      ptr_path_time_graph_(ptr_path_time_graph),
      ptr_prediction_obstacles_(ptr_prediction_obstacles) {}

std::vector<SamplePoint>
ConditionFilter::QueryPathTimeObstacleSamplePoints() const {
  std::vector<SamplePoint> sample_points;
  for (const auto& path_time_obstacle :
       ptr_path_time_graph_->GetPathTimeObstacles()) {
    std::string obstacle_id = path_time_obstacle.obstacle_id();

    std::vector<PathTimePoint> overtake_path_time_points =
        ptr_path_time_graph_->GetObstacleSurroundingPoints(
            obstacle_id, FLAGS_lattice_epsilon, FLAGS_time_min_density);
    for (const PathTimePoint& path_time_point : overtake_path_time_points) {
      double v = ptr_prediction_obstacles_->ProjectVelocityAlongReferenceLine(
          obstacle_id, path_time_point.s(), path_time_point.t());
      SamplePoint sample_point;
      sample_point.mutable_path_time_point()->CopyFrom(path_time_point);
      sample_point.mutable_path_time_point()->set_s(FLAGS_default_lon_buffer);
      sample_point.set_ref_v(v);
      sample_points.push_back(std::move(sample_point));
    }

    std::vector<PathTimePoint> follow_path_time_points =
        ptr_path_time_graph_->GetObstacleSurroundingPoints(
            obstacle_id, -FLAGS_lattice_epsilon, FLAGS_time_min_density);
    for (const PathTimePoint& path_time_point : follow_path_time_points) {
      double v = ptr_prediction_obstacles_->ProjectVelocityAlongReferenceLine(
          obstacle_id, path_time_point.s(), path_time_point.t());
      SamplePoint sample_point;
      sample_point.mutable_path_time_point()->CopyFrom(path_time_point);
      sample_point.mutable_path_time_point()->set_s(-FLAGS_default_lon_buffer);
      sample_point.set_ref_v(v);
      sample_points.push_back(std::move(sample_point));
    }
  }
  return sample_points;
}

// Compute pixel img for lattice st
bool ConditionFilter::GenerateLatticeStPixels(
    apollo::planning_internal::LatticeStTraining* st_data, double timestamp,
    std::string st_img_name) const {
  int num_rows = 250;
  int num_cols = 160;
  double s_step = 100.0 / static_cast<double>(num_rows);
  double t_step = 8.0 / static_cast<double>(num_cols);

  if (ptr_path_time_graph_->GetPathTimeObstacles().empty()) {
    ADEBUG << "No_Path_Time_Neighborhood_Obstacle_in_this_frame";
    return false;
  }
  for (int j = 0; j < num_cols; ++j) {
    double t = t_step * (j + 1);
    double feasible_s_upper = feasible_region_.SUpper(t);
    double feasible_s_lower = feasible_region_.SLower(t);
    for (int i = 0; i < num_rows; ++i) {
      double s = s_step * (num_rows - i + 1);
      if (s <= feasible_s_lower || s >= feasible_s_upper) {
        // Dye gray
        apollo::planning_internal::LatticeStPixel* pixel = st_data->add_pixel();
        pixel->set_s(i);
        pixel->set_t(j);
        pixel->set_r(128);
        pixel->set_g(128);
        pixel->set_b(128);
        continue;
      }
      if (WithinObstacleSt(s, t)) {
        // Dye blue
        apollo::planning_internal::LatticeStPixel* pixel = st_data->add_pixel();
        pixel->set_s(i);
        pixel->set_t(j);
        pixel->set_r(0);
        pixel->set_g(0);
        pixel->set_b(128);
        continue;
      }
    }
  }
  st_data->set_annotation(st_img_name);
  st_data->set_timestamp(timestamp);
  st_data->set_num_s_grids(num_rows);
  st_data->set_num_t_grids(num_cols);
  st_data->set_s_resolution(s_step);
  st_data->set_t_resolution(t_step);
  return true;
}

bool ConditionFilter::WithinObstacleSt(double s, double t) const {
  const auto& path_time_obstacles =
      ptr_path_time_graph_->GetPathTimeObstacles();

  for (const PathTimeObstacle& path_time_obstacle : path_time_obstacles) {
    if (t < path_time_obstacle.upper_left().t() ||
        t > path_time_obstacle.upper_right().t()) {
      continue;
    }

    double s_upper = lerp(path_time_obstacle.upper_left().s(),
                          path_time_obstacle.upper_left().t(),
                          path_time_obstacle.upper_right().s(),
                          path_time_obstacle.upper_right().t(), t);
    double s_lower = lerp(path_time_obstacle.bottom_left().s(),
                          path_time_obstacle.bottom_left().t(),
                          path_time_obstacle.bottom_right().s(),
                          path_time_obstacle.bottom_right().t(), t);

    if (s <= s_upper && s >= s_lower) {
      return true;
    }
  }
  return false;
}

}  // namespace planning
}  // namespace apollo
