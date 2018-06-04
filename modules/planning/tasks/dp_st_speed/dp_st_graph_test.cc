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
#include "modules/planning/tasks/dp_st_speed/dp_st_graph.h"

#include "gtest/gtest.h"

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/planning/proto/planning_config.pb.h"

#include "modules/common/log.h"
#include "modules/common/util/file.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/planning_thread_pool.h"

namespace apollo {
namespace planning {

class DpStGraphTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    PlanningThreadPool::instance()->Init();

    // dp_config_
    PlanningConfig config;
    FLAGS_planning_config_file = "modules/planning/conf/planning_config.pb.txt";
    CHECK(apollo::common::util::GetProtoFromFile(FLAGS_planning_config_file,
                                                 &config));
    dp_config_ = config.em_planner_config().dp_st_speed_config();

    // speed_limit:
    for (float s = 0; s < 200.0; s += 1.0) {
      speed_limit_.AppendSpeedLimit(s, 25.0);
    }
  }

  std::list<PathObstacle> path_obstacle_list_;
  std::list<Obstacle> obstacle_list_;

  StGraphData st_graph_data_;
  SpeedLimit speed_limit_;

  DpStSpeedConfig dp_config_;

  common::TrajectoryPoint init_point_;
  SLBoundary adc_sl_boundary_;
};

TEST_F(DpStGraphTest, simple) {
  Obstacle o1;
  o1.SetId("o1");
  obstacle_list_.push_back(o1);
  path_obstacle_list_.emplace_back(&(obstacle_list_.back()));

  std::vector<const PathObstacle*> obstacles_;
  obstacles_.emplace_back(&(path_obstacle_list_.back()));

  std::vector<STPoint> upper_points;
  std::vector<STPoint> lower_points;
  std::vector<std::pair<STPoint, STPoint>> point_pairs;

  lower_points.emplace_back(30.0, 4.0);
  lower_points.emplace_back(30.0, 6.0);
  upper_points.emplace_back(45.0, 4.0);
  upper_points.emplace_back(45.0, 6.0);

  point_pairs.emplace_back(lower_points[0], upper_points[0]);
  point_pairs.emplace_back(lower_points[1], upper_points[1]);

  path_obstacle_list_.back().SetStBoundary(StBoundary(point_pairs));

  std::vector<const StBoundary*> boundaries;
  boundaries.push_back(&(obstacles_.back()->st_boundary()));

  init_point_.mutable_path_point()->set_x(0.0);
  init_point_.mutable_path_point()->set_y(0.0);
  init_point_.mutable_path_point()->set_z(0.0);
  init_point_.mutable_path_point()->set_kappa(0.0);
  init_point_.set_v(10.0);
  init_point_.set_a(0.0);

  const float path_data_length = 120.0;

  st_graph_data_ =
      StGraphData(boundaries, init_point_, speed_limit_, path_data_length);

  // adc_sl_boundary_
  adc_sl_boundary_.set_start_s(15.0);
  adc_sl_boundary_.set_end_s(20.0);
  adc_sl_boundary_.set_start_l(-1.1);
  adc_sl_boundary_.set_end_l(1.1);

  DpStGraph dp_st_graph(st_graph_data_, dp_config_, obstacles_, init_point_,
                        adc_sl_boundary_);

  SpeedData speed_data;
  auto ret = dp_st_graph.Search(&speed_data);
  EXPECT_TRUE(ret.ok());
}

}  // namespace planning
}  // namespace apollo
