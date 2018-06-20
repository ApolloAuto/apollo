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
 * @brief This file provides several unit tests for the class
 * "NaviObstacleDecider".
 */

#include "modules/planning/navi/decider/navi_obstacle_decider.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/localization/common/localization_gflags.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/navi/common/local_path.h"

using apollo::common::PathPoint;
using apollo::common::util::MakePathPoint;
using apollo::perception::PerceptionObstacle;
using std::vector;

namespace apollo {
namespace planning {

TEST(NaviObstacleDeciderTest, ComputeObstacleDist) {
  std::vector<const Obstacle*> vec_obstack;
  std::vector<common::PathPoint> vec_points;

  PerceptionObstacle perception_obstacle;
  perception_obstacle.set_width(1.0);
  perception_obstacle.set_length(1.0);
  perception_obstacle.mutable_position()->set_x(1.0);
  perception_obstacle.mutable_position()->set_y(1.0);
  Obstacle b1("1", perception_obstacle);

  PathPoint p1 = MakePathPoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  PathPoint p2 = MakePathPoint(0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  vec_points.push_back(p1);
  vec_points.push_back(p2);
  LocalPath fpath(vec_points);
  vec_obstack.push_back(&b1);
  NaviObstacleDecider navi_obstacle;
  navi_obstacle.ProcessPathObstacle(vec_obstack, &fpath);

  auto& obstacle_lat_dist = navi_obstacle.MutableObstacleLatDistance();
  std::map<double, double>::iterator iter = obstacle_lat_dist.begin();
  EXPECT_FLOAT_EQ(iter->first, 1.0);
  EXPECT_FLOAT_EQ(iter->second, 1.0);
}

TEST(NaviObstacleDeciderTest, ComputeLeftandrightNudgableDist1) {
  float left_nudgable = 0.0;
  float right_nudgable = 0.0;
  NaviObstacleDecider get_path_nudge_distance;

  PathPoint p1 = MakePathPoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  std::vector<common::PathPoint> path_point;
  path_point.push_back(p1);
  LocalPath fpath(path_point);

  get_path_nudge_distance.GetLeftRightNudgableDistance(&fpath, &left_nudgable,
                                                       &right_nudgable);

  EXPECT_FLOAT_EQ(left_nudgable, 0.595);
  EXPECT_FLOAT_EQ(right_nudgable, 0.595);
}

TEST(NaviObstacleDeciderTest, ComputeLeftandrightNudgableDist2) {
  float left_nudgable = 0.0;
  float right_nudgable = 0.0;
  NaviObstacleDecider get_path_nudge_distance;

  PathPoint p2 = MakePathPoint(1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  std::vector<common::PathPoint> path_point;
  path_point.push_back(p2);
  LocalPath fpath(path_point);

  get_path_nudge_distance.GetLeftRightNudgableDistance(&fpath, &left_nudgable,
                                                       &right_nudgable);

  EXPECT_FLOAT_EQ(left_nudgable, 1.595);
  EXPECT_FLOAT_EQ(right_nudgable, -0.405);
}

TEST(NaviObstacleDeciderTest, ComputeNudgeDist) {
  NaviObstacleDecider obstacledecider;
  auto& obstacle_lat_dist = obstacledecider.MutableObstacleLatDistance();

  obstacle_lat_dist.insert(std::pair<double, float>(2.0, 2.5));
  float nudge_dist = 0;
  nudge_dist = obstacledecider.GetNudgeDistance(1.0, -0.5);
  EXPECT_FLOAT_EQ(nudge_dist, -0.455);
}

TEST(NaviObstacleDeciderTest, ComputeNudgeDist1) {
  NaviObstacleDecider obstacledecider;
  auto& obstacle_lat_dist = obstacledecider.MutableObstacleLatDistance();

  obstacle_lat_dist.insert(std::pair<double, float>(2.0, -2.5));
  float nudge_dist = 0.0;
  nudge_dist = obstacledecider.GetNudgeDistance(1.0, 0.0);
  EXPECT_FLOAT_EQ(nudge_dist, 0.455);
}

}  // namespace planning
}  // namespace apollo
