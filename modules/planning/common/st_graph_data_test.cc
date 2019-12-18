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

#include "modules/planning/common/st_graph_data.h"

#include "gmock/gmock.h"

#include "cyber/common/log.h"
#include "modules/planning/common/speed/st_boundary.h"

namespace apollo {
namespace planning {

TEST(StGraphDataTest, basic_test) {
  std::vector<const STBoundary*> boundary_vec;
  auto boundary = STBoundary();
  boundary_vec.push_back(&boundary);
  apollo::common::TrajectoryPoint traj_point;
  traj_point.mutable_path_point()->set_x(1.1);
  traj_point.mutable_path_point()->set_y(2.1);
  traj_point.mutable_path_point()->set_theta(0.2);
  traj_point.mutable_path_point()->set_kappa(0.02);
  traj_point.mutable_path_point()->set_dkappa(0.123);
  traj_point.mutable_path_point()->set_ddkappa(0.003);
  traj_point.set_v(10.001);
  traj_point.set_a(1.022);
  traj_point.set_relative_time(1010.022);

  SpeedLimit speed_limit;
  double cruise_speed = 5.0;
  double path_data_length = 100.0;
  double total_time_by_conf = 7.0;
  planning_internal::STGraphDebug* st_graph_debug = nullptr;
  StGraphData st_graph_data;
  st_graph_data.LoadData(boundary_vec, 0.0, traj_point, speed_limit,
                         cruise_speed, path_data_length, total_time_by_conf,
                         st_graph_debug);
  EXPECT_EQ(st_graph_data.st_boundaries().size(), 1);
  EXPECT_DOUBLE_EQ(st_graph_data.init_point().path_point().x(), 1.1);
  EXPECT_DOUBLE_EQ(st_graph_data.init_point().path_point().y(), 2.1);
  EXPECT_DOUBLE_EQ(st_graph_data.init_point().path_point().z(), 0.0);
  EXPECT_DOUBLE_EQ(st_graph_data.init_point().path_point().theta(), 0.2);
  EXPECT_DOUBLE_EQ(st_graph_data.init_point().path_point().kappa(), 0.02);
  EXPECT_DOUBLE_EQ(st_graph_data.init_point().path_point().dkappa(), 0.123);
  EXPECT_DOUBLE_EQ(st_graph_data.init_point().path_point().ddkappa(), 0.003);
  EXPECT_DOUBLE_EQ(st_graph_data.init_point().v(), 10.001);
  EXPECT_DOUBLE_EQ(st_graph_data.init_point().a(), 1.022);
  EXPECT_DOUBLE_EQ(st_graph_data.init_point().relative_time(), 1010.022);
  EXPECT_DOUBLE_EQ(st_graph_data.path_length(), 100.0);
}

}  // namespace planning
}  // namespace apollo
