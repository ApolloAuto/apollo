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

#include "modules/planning/common/em_planning_data.h"
#include "glog/logging.h"

#include "modules/common/proto/path_point.pb.h"
#include "modules/planning/math/double.h"

namespace apollo {
namespace planning {

std::string EMPlanningData::type() const {
  return "EMPlanningData";
}

void EMPlanningData::init(const std::size_t num_iter) {
  _num_iter = num_iter;
  _path_data_vec = std::vector < PathData > (num_iter, PathData());
  _speed_data_vec = std::vector < SpeedData > (num_iter + 1, SpeedData());
}

std::size_t EMPlanningData::num_iter() const {
  return _num_iter;
}

const PathData& EMPlanningData::path_data(const std::size_t index) const {
  return _path_data_vec[index];
}

const SpeedData& EMPlanningData::speed_data(const std::size_t index) const {
  return _speed_data_vec[index];
}

PathData* EMPlanningData::mutable_path_data(const std::size_t index) {
  if (index >= _path_data_vec.size()) {
    return nullptr;
  }
  return &_path_data_vec[index];
}

SpeedData* EMPlanningData::mutable_speed_data(const std::size_t index) {
  if (index >= _speed_data_vec.size()) {
    return nullptr;
  }
  return &_speed_data_vec[index];
}

bool EMPlanningData::aggregate(const double time_resolution) {
  CHECK(time_resolution > 0.0);
  CHECK(_computed_trajectory.num_of_points() == 0);

  const SpeedData& speed_data = _speed_data_vec.back();
  const PathData& path_data = _path_data_vec.back();
  for (double cur_rel_time = 0.0; cur_rel_time < speed_data.total_time();
      cur_rel_time += time_resolution) {

    SpeedPoint speed_point;
    QUIT_IF(!speed_data.get_speed_point_with_time(cur_rel_time, &speed_point), false,
        ERROR, "Fail to get speed point with relative time %f", cur_rel_time);

    apollo::common::PathPoint path_point;
    // TODO temp fix speed point s out of path point bound, need further refine later
    if (speed_point.s() > path_data.path().param_length()) {
        break;
    }
    QUIT_IF(!path_data.get_path_point_with_path_s(speed_point.s(), &path_point), false,
        ERROR, "Fail to get path data with s %f, path total length %f",
        speed_point.s(), path_data.path().param_length());

    apollo::common::TrajectoryPoint trajectory_point;
    trajectory_point.mutable_path_point()->CopyFrom(path_point);
    trajectory_point.set_v(speed_point.v());
    trajectory_point.set_a(speed_point.a());
    trajectory_point.set_relative_time(
        _init_planning_point.relative_time() + speed_point.t());
    _computed_trajectory.add_trajectory_point(trajectory_point);
  }
  return true;
}
}  // namespace planning
}  // namespace apollo
