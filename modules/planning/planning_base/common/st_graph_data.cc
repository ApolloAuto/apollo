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
 * @file: st_graph_data.cc
 **/

#include "modules/planning/planning_base/common/st_graph_data.h"

#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;

void StGraphData::LoadData(const std::vector<const STBoundary*>& st_boundaries,
                           const double min_s_on_st_boundaries,
                           const apollo::common::TrajectoryPoint& init_point,
                           const SpeedLimit& speed_limit,
                           const double cruise_speed,
                           const double path_data_length,
                           const double total_time_by_conf,
                           planning_internal::STGraphDebug* st_graph_debug) {
  init_ = true;
  st_boundaries_ = st_boundaries;
  min_s_on_st_boundaries_ = min_s_on_st_boundaries;
  init_point_ = init_point;
  speed_limit_ = speed_limit;
  cruise_speed_ = cruise_speed;
  path_data_length_ = path_data_length;
  total_time_by_conf_ = total_time_by_conf;
  st_graph_debug_ = st_graph_debug;
}

const std::vector<const STBoundary*>& StGraphData::st_boundaries() const {
  return st_boundaries_;
}

double StGraphData::min_s_on_st_boundaries() const {
  return min_s_on_st_boundaries_;
}

const TrajectoryPoint& StGraphData::init_point() const { return init_point_; }

const SpeedLimit& StGraphData::speed_limit() const { return speed_limit_; }

double StGraphData::cruise_speed() const {
  return cruise_speed_ > 0.0 ? cruise_speed_ : FLAGS_default_cruise_speed;
}

double StGraphData::path_length() const { return path_data_length_; }

double StGraphData::total_time_by_conf() const { return total_time_by_conf_; }

planning_internal::STGraphDebug* StGraphData::mutable_st_graph_debug() {
  return st_graph_debug_;
}

bool StGraphData::SetSTDrivableBoundary(
    const std::vector<std::tuple<double, double, double>>& s_boundary,
    const std::vector<std::tuple<double, double, double>>& v_obs_info) {
  if (s_boundary.size() != v_obs_info.size()) {
    return false;
  }
  for (size_t i = 0; i < s_boundary.size(); ++i) {
    auto st_bound_instance = st_drivable_boundary_.add_st_boundary();
    st_bound_instance->set_t(std::get<0>(s_boundary[i]));
    st_bound_instance->set_s_lower(std::get<1>(s_boundary[i]));
    st_bound_instance->set_s_upper(std::get<2>(s_boundary[i]));
    if (std::get<1>(v_obs_info[i]) > -kObsSpeedIgnoreThreshold) {
      st_bound_instance->set_v_obs_lower(std::get<1>(v_obs_info[i]));
    }
    if (std::get<2>(v_obs_info[i]) < kObsSpeedIgnoreThreshold) {
      st_bound_instance->set_v_obs_upper(std::get<2>(v_obs_info[i]));
    }
  }
  return true;
}

const STDrivableBoundary& StGraphData::st_drivable_boundary() const {
  return st_drivable_boundary_;
}

}  // namespace planning
}  // namespace apollo
