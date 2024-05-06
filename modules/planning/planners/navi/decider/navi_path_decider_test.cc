/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
 * @brief This file provides several unit tests for the class "NaviPathDecider".
 */

#include "modules/planning/planners/navi/decider/navi_path_decider.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "modules/common/util/point_factory.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

using apollo::common::util::PointFactory;

namespace apollo {
namespace planning {
class NaviPathDeciderTest : public ::testing::Test {
 public:
  static void SetUpTestCase() {
    AINFO << "NaviPathDeciderTest : SetUpTestCase";
  }

  static void GeneratePathData(
      double s, double init_y, double kappa,
      std::vector<common::PathPoint>* const path_points) {
    for (double x = 0.0, y = init_y; x < s; ++x) {
      path_points->clear();
      path_points->push_back(
          PointFactory::ToPathPoint(x, y, 0.0, 0.0, 0.0, kappa));
    }
  }

  static void InitPlannigConfig(PlannerNaviConfig* const plannig_config) {
    auto* navi_path_decider_config =
        plannig_config->mutable_navi_path_decider_config();
    navi_path_decider_config->set_min_path_length(5.0);
    navi_path_decider_config->set_min_look_forward_time(2.0);
    navi_path_decider_config->set_max_keep_lane_distance(0.4);
    navi_path_decider_config->set_max_keep_lane_shift_y(0.15);
    navi_path_decider_config->set_min_keep_lane_offset(0.20);
    navi_path_decider_config->set_keep_lane_shift_compensation(0.01);
    navi_path_decider_config->set_move_dest_lane_compensation(0.35);
    navi_path_decider_config->clear_move_dest_lane_config_talbe();
    auto* move_dest_lane_cfg_table =
        navi_path_decider_config->mutable_move_dest_lane_config_talbe();
    auto* move_shift_config = move_dest_lane_cfg_table->add_lateral_shift();
    move_shift_config->set_max_speed(34);
    move_shift_config->set_max_move_dest_lane_shift_y(0.45);
  }
};

TEST_F(NaviPathDeciderTest, Init) {
  NaviPathDecider navi_path_decider;
  PlannerNaviConfig config;
  InitPlannigConfig(&config);
  EXPECT_TRUE(navi_path_decider.Init(config));
}

TEST_F(NaviPathDeciderTest, Execute) {}

TEST_F(NaviPathDeciderTest, MoveToDestLane) {
  NaviPathDecider navi_path_decider;
  PlannerNaviConfig config;
  InitPlannigConfig(&config);
  navi_path_decider.Init(config);

  // generate path point
  static constexpr double kMaxS = 152.0;
  std::vector<common::PathPoint> path_points;

  // 1.std::fabs(target_path_init_y) < max_keep_lane_distance not need move to
  // dest lane
  GeneratePathData(kMaxS, 0.03, 0.03, &path_points);
  double dest_y = path_points[0].y();
  double expect_y = path_points[0].y();
  navi_path_decider.MoveToDestLane(dest_y, &path_points);
  EXPECT_DOUBLE_EQ(path_points[0].y(), expect_y);

  // 2.std::fabs(target_path_init_y) > max_keep_lane_distance need move to dest
  // lane 2.1 move to left, not need compensation
  navi_path_decider.start_plan_v_ = 10.0;
  navi_path_decider.start_plan_a_ = 1.0;
  GeneratePathData(kMaxS, 0.9, 0.03, &path_points);
  dest_y = path_points[0].y();
  auto navi_path_decider_cfg = config.navi_path_decider_config();
  expect_y = navi_path_decider_cfg.move_dest_lane_config_talbe()
                 .lateral_shift(0)
                 .max_move_dest_lane_shift_y();
  navi_path_decider.MoveToDestLane(dest_y, &path_points);
  EXPECT_DOUBLE_EQ(path_points[0].y(), expect_y);

  // 2.std::fabs(target_path_init_y) > max_keep_lane_distance need move to dest
  // lane 2.2 move to right, need compensation
  navi_path_decider.start_plan_v_ = 10.0;
  navi_path_decider.start_plan_a_ = 1.0;
  GeneratePathData(kMaxS, -0.9, 0.03, &path_points);
  dest_y = path_points[0].y();
  expect_y = -navi_path_decider_cfg.move_dest_lane_config_talbe()
                  .lateral_shift(0)
                  .max_move_dest_lane_shift_y() -
             navi_path_decider_cfg.move_dest_lane_compensation();
  navi_path_decider.MoveToDestLane(dest_y, &path_points);
  EXPECT_DOUBLE_EQ(path_points[0].y(), expect_y);
}

TEST_F(NaviPathDeciderTest, KeepLane) {
  NaviPathDecider navi_path_decider;
  PlannerNaviConfig config;
  InitPlannigConfig(&config);
  navi_path_decider.Init(config);

  // generate path point
  static constexpr double kMaxS = 152.0;
  std::vector<common::PathPoint> path_points;

  // 1.std::fabs(target_path_init_y) > max_keep_lane_distance not need keep lane
  GeneratePathData(kMaxS, 0.90, 0.03, &path_points);
  double dest_y = path_points[0].y();
  double expect_y = path_points[0].y();
  navi_path_decider.KeepLane(dest_y, &path_points);
  EXPECT_DOUBLE_EQ(path_points[0].y(), expect_y);

  // 2. std::fabs(target_path_init_y)<= max_keep_lane_distance need keep lane
  // 2.1 std::fabs(target_path_init_y) < keep_lane_offset, not need adjust
  // reference points
  const common::TrajectoryPoint plan_start_point;
  const common::VehicleState vehicle_state;
  ReferenceLine ref_line;
  apollo::hdmap::RouteSegments route_segments;
  navi_path_decider.reference_line_info_ = new ReferenceLineInfo(
      vehicle_state, plan_start_point, ref_line, route_segments);
  LocalView local_view;
  navi_path_decider.frame_ =
      new Frame(1, local_view, plan_start_point, vehicle_state, nullptr);
  CHECK_NOTNULL(navi_path_decider.reference_line_info_);
  CHECK_NOTNULL(navi_path_decider.frame_);
  GeneratePathData(kMaxS, 0.19, 0.03, &path_points);
  dest_y = path_points[0].y();
  expect_y = path_points[0].y();
  navi_path_decider.KeepLane(dest_y, &path_points);
  EXPECT_DOUBLE_EQ(path_points[0].y(), expect_y);

  // 2.2 min_keep_lane_offset + max_keep_lane_shift_y -
  // keep_lane_shift_compensation > std::fabs(target_path_init_y) >
  // min_keep_lane_offset, need adjust reference points
  GeneratePathData(kMaxS, 0.29, 0.03, &path_points);
  dest_y = path_points[0].y();
  auto navi_path_decider_cfg = config.navi_path_decider_config();
  expect_y = dest_y - navi_path_decider_cfg.min_keep_lane_offset() +
             navi_path_decider_cfg.keep_lane_shift_compensation() + dest_y;
  navi_path_decider.KeepLane(dest_y, &path_points);
  EXPECT_DOUBLE_EQ(path_points[0].y(), expect_y);

  // 2.2 min_keep_lane_offset + max_keep_lane_shift_y -
  // keep_lane_shift_compensation <= std::fabs(target_path_init_y)
  // min_keep_lane_offset, need adjust reference points
  GeneratePathData(kMaxS, 0.34, 0.03, &path_points);
  dest_y = path_points[0].y();
  expect_y = path_points[0].y();
  expect_y = navi_path_decider_cfg.max_keep_lane_shift_y() + dest_y;
  navi_path_decider.KeepLane(dest_y, &path_points);
  EXPECT_DOUBLE_EQ(path_points[0].y(), expect_y);

  delete navi_path_decider.reference_line_info_;
  navi_path_decider.reference_line_info_ = nullptr;
  delete navi_path_decider.frame_;
  navi_path_decider.frame_ = nullptr;
}

}  // namespace planning
}  // namespace apollo
