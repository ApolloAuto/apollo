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
#include "modules/planning/tasks/optimizers/path_time_heuristic/gridded_path_time_graph.h"

#include "gtest/gtest.h"

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/planning/proto/planning_config.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::cyber::common::GetProtoFromFile;

class DpStGraphTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    FLAGS_enable_multi_thread_in_dp_st_graph = false;
    FLAGS_scenario_lane_follow_config_file =
        "/apollo/modules/planning/conf/scenario/lane_follow_config.pb.txt";
    ScenarioConfig config;
    ACHECK(GetProtoFromFile(FLAGS_scenario_lane_follow_config_file, &config));

    FLAGS_planning_config_file =
        "/apollo/modules/planning/conf/planning_config.pb.txt";
    PlanningConfig planning_config;
    ACHECK(GetProtoFromFile(FLAGS_planning_config_file, &planning_config))
        << "failed to load planning config file " << FLAGS_planning_config_file;

    DpStSpeedConfig default_dp_config;
    for (const auto& cfg : planning_config.default_task_config()) {
      if (cfg.task_type() == TaskConfig::DP_ST_SPEED_OPTIMIZER) {
        default_dp_config = cfg.speed_heuristic_config().default_speed_config();
        break;
      }
    }
    dp_config_ = default_dp_config;

    for (const auto& stage : config.stage_config()) {
      for (const auto& cfg : stage.task_config()) {
        if (cfg.task_type() == TaskConfig::DP_ST_SPEED_OPTIMIZER) {
          dp_config_.MergeFrom(
              cfg.speed_heuristic_config().default_speed_config());
          break;
        }
      }
    }

    AERROR << dp_config_.ShortDebugString();

    // speed_limit:
    for (double s = 0; s < 200.0; s += 1.0) {
      speed_limit_.AppendSpeedLimit(s, 25.0);
    }
  }

  virtual void TearDown() {}

 protected:
  std::list<Obstacle> obstacle_list_;

  StGraphData st_graph_data_;
  SpeedLimit speed_limit_;

  DpStSpeedConfig dp_config_;

  common::TrajectoryPoint init_point_;
};

TEST_F(DpStGraphTest, simple) {
  Obstacle o1;
  o1.SetId("o1");
  obstacle_list_.push_back(o1);

  std::vector<const Obstacle*> obstacles_;
  obstacles_.emplace_back(&(obstacle_list_.back()));

  std::vector<STPoint> upper_points;
  std::vector<STPoint> lower_points;
  std::vector<std::pair<STPoint, STPoint>> point_pairs;

  lower_points.emplace_back(30.0, 4.0);
  lower_points.emplace_back(30.0, 6.0);
  upper_points.emplace_back(45.0, 4.0);
  upper_points.emplace_back(45.0, 6.0);

  point_pairs.emplace_back(lower_points[0], upper_points[0]);
  point_pairs.emplace_back(lower_points[1], upper_points[1]);

  obstacle_list_.back().set_path_st_boundary(STBoundary(point_pairs));

  std::vector<const STBoundary*> boundaries;
  boundaries.push_back(&(obstacles_.back()->path_st_boundary()));

  init_point_.mutable_path_point()->set_x(0.0);
  init_point_.mutable_path_point()->set_y(0.0);
  init_point_.mutable_path_point()->set_z(0.0);
  init_point_.mutable_path_point()->set_kappa(0.0);
  init_point_.set_v(10.0);
  init_point_.set_a(0.0);

  planning_internal::STGraphDebug st_graph_debug;

  st_graph_data_ = StGraphData();
  st_graph_data_.LoadData(boundaries, 30.0, init_point_, speed_limit_, 5.0,
                          120.0, 7.0, &st_graph_debug);

  GriddedPathTimeGraph dp_st_graph(st_graph_data_, dp_config_, obstacles_,
                                   init_point_);

  SpeedData speed_data;
  auto ret = dp_st_graph.Search(&speed_data);
  EXPECT_TRUE(ret.ok());
}

}  // namespace planning
}  // namespace apollo
