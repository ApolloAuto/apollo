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
 * @file: st_graph_point.cc
 **/

#include "modules/planning/tasks/path_time_heuristic/st_graph_point.h"

#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

std::uint32_t StGraphPoint::index_s() const { return index_s_; }

std::uint32_t StGraphPoint::index_t() const { return index_t_; }

const STPoint& StGraphPoint::point() const { return point_; }

const StGraphPoint* StGraphPoint::pre_point() const { return pre_point_; }

double StGraphPoint::reference_cost() const { return reference_cost_; }

double StGraphPoint::obstacle_cost() const { return obstacle_cost_; }

double StGraphPoint::spatial_potential_cost() const {
  return spatial_potential_cost_;
}

double StGraphPoint::total_cost() const { return total_cost_; }

void StGraphPoint::Init(const std::uint32_t index_t,
                        const std::uint32_t index_s, const STPoint& st_point) {
  index_t_ = index_t;
  index_s_ = index_s;
  point_ = st_point;
}

void StGraphPoint::SetReferenceCost(const double reference_cost) {
  reference_cost_ = reference_cost;
}

void StGraphPoint::SetObstacleCost(const double obs_cost) {
  obstacle_cost_ = obs_cost;
}

void StGraphPoint::SetSpatialPotentialCost(
    const double spatial_potential_cost) {
  spatial_potential_cost_ = spatial_potential_cost;
}

void StGraphPoint::SetTotalCost(const double total_cost) {
  total_cost_ = total_cost;
}

void StGraphPoint::SetPrePoint(const StGraphPoint& pre_point) {
  pre_point_ = &pre_point;
}

double StGraphPoint::GetOptimalSpeed() const { return optimal_speed_; }

void StGraphPoint::SetOptimalSpeed(const double optimal_speed) {
  optimal_speed_ = optimal_speed;
}

}  // namespace planning
}  // namespace apollo
