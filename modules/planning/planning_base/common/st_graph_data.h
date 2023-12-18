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
 * @file: st_graph_data.h
 * @brief: data with map info and obstacle info
 **/

#pragma once

#include <tuple>
#include <vector>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/planning/planning_base/proto/st_drivable_boundary.pb.h"

#include "modules/planning/planning_base/common/speed/st_boundary.h"
#include "modules/planning/planning_base/common/speed_limit.h"

namespace apollo {
namespace planning {

constexpr double kObsSpeedIgnoreThreshold = 100.0;

class StGraphData {
 public:
  StGraphData() = default;

  void LoadData(const std::vector<const STBoundary*>& st_boundaries,
                const double min_s_on_st_boundaries,
                const apollo::common::TrajectoryPoint& init_point,
                const SpeedLimit& speed_limit, const double cruise_speed,
                const double path_data_length, const double total_time_by_conf,
                planning_internal::STGraphDebug* st_graph_debug);

  bool is_initialized() const { return init_; }

  const std::vector<const STBoundary*>& st_boundaries() const;

  double min_s_on_st_boundaries() const;

  const apollo::common::TrajectoryPoint& init_point() const;

  const SpeedLimit& speed_limit() const;

  double cruise_speed() const;

  double path_length() const;

  double total_time_by_conf() const;

  planning_internal::STGraphDebug* mutable_st_graph_debug();

  bool SetSTDrivableBoundary(
      const std::vector<std::tuple<double, double, double>>& s_boundary,
      const std::vector<std::tuple<double, double, double>>& v_obs_info);

  const STDrivableBoundary& st_drivable_boundary() const;

 private:
  bool init_ = false;
  std::vector<const STBoundary*> st_boundaries_;
  double min_s_on_st_boundaries_ = 0.0;
  apollo::common::TrajectoryPoint init_point_;
  SpeedLimit speed_limit_;
  double cruise_speed_ = 0.0;
  double path_data_length_ = 0.0;
  double path_length_by_conf_ = 0.0;
  double total_time_by_conf_ = 0.0;
  planning_internal::STGraphDebug* st_graph_debug_ = nullptr;

  STDrivableBoundary st_drivable_boundary_;
};

}  // namespace planning
}  // namespace apollo
