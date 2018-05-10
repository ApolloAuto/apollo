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

#include <memory>
#include <unordered_map>
#include <vector>

#include "gtest/gtest.h"

#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"

#include "modules/common/util/file.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::perception::PerceptionObstacle;

TEST(Obstacle, IsStaticObstacle) {
  PerceptionObstacle perception_obstacle;
  EXPECT_TRUE(Obstacle::IsStaticObstacle(perception_obstacle));

  perception_obstacle.mutable_velocity()->set_x(2.5);
  perception_obstacle.mutable_velocity()->set_y(0.5);
  EXPECT_FALSE(Obstacle::IsStaticObstacle(perception_obstacle));

  perception_obstacle.set_type(PerceptionObstacle::UNKNOWN);
  EXPECT_FALSE(Obstacle::IsStaticObstacle(perception_obstacle));

  perception_obstacle.set_type(PerceptionObstacle::UNKNOWN_UNMOVABLE);
  EXPECT_TRUE(Obstacle::IsStaticObstacle(perception_obstacle));

  perception_obstacle.set_type(PerceptionObstacle::UNKNOWN_MOVABLE);
  EXPECT_FALSE(Obstacle::IsStaticObstacle(perception_obstacle));

  perception_obstacle.set_type(PerceptionObstacle::PEDESTRIAN);
  EXPECT_FALSE(Obstacle::IsStaticObstacle(perception_obstacle));

  perception_obstacle.set_type(PerceptionObstacle::BICYCLE);
  EXPECT_FALSE(Obstacle::IsStaticObstacle(perception_obstacle));

  perception_obstacle.set_type(PerceptionObstacle::VEHICLE);
  EXPECT_FALSE(Obstacle::IsStaticObstacle(perception_obstacle));

  perception_obstacle.mutable_velocity()->set_x(0.5);
  perception_obstacle.mutable_velocity()->set_y(0.5);
  EXPECT_TRUE(Obstacle::IsStaticObstacle(perception_obstacle));
}

class ObstacleTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    prediction::PredictionObstacles prediction_obstacles;
    ASSERT_TRUE(common::util::GetProtoFromFile(
        "modules/planning/common/testdata/sample_prediction.pb.txt",
        &prediction_obstacles));
    auto obstacles = Obstacle::CreateObstacles(prediction_obstacles);
    ASSERT_EQ(5, obstacles.size());
    for (auto& obstacle : obstacles) {
      const auto id = obstacle->Id();
      indexed_obstacles_.Add(id, *obstacle);
    }
  }

 protected:
  IndexedObstacles indexed_obstacles_;
};

TEST_F(ObstacleTest, CreateObstacles) {
  ASSERT_EQ(5, indexed_obstacles_.Items().size());
  EXPECT_TRUE(indexed_obstacles_.Find("2156_0"));
  EXPECT_TRUE(indexed_obstacles_.Find("2156_1"));
  EXPECT_TRUE(indexed_obstacles_.Find("2157_0"));
  EXPECT_TRUE(indexed_obstacles_.Find("2157_1"));
  EXPECT_TRUE(indexed_obstacles_.Find("2161"));
}

TEST_F(ObstacleTest, Id) {
  const auto* obstacle = indexed_obstacles_.Find("2156_0");
  ASSERT_TRUE(obstacle);
  EXPECT_EQ("2156_0", obstacle->Id());
  EXPECT_EQ(2156, obstacle->PerceptionId());
}

TEST_F(ObstacleTest, GetPointAtTime) {
  const auto* obstacle = indexed_obstacles_.Find("2156_0");
  ASSERT_TRUE(obstacle);

  // first
  const auto first_point = obstacle->GetPointAtTime(0.0);
  EXPECT_FLOAT_EQ(0.0, first_point.relative_time());
  EXPECT_FLOAT_EQ(76.684071405, first_point.path_point().x());
  EXPECT_FLOAT_EQ(350.481852505, first_point.path_point().y());

  // last
  const auto last_point = obstacle->GetPointAtTime(10.04415320);
  EXPECT_FLOAT_EQ(10.0441531943, last_point.relative_time());
  EXPECT_FLOAT_EQ(186.259371951, last_point.path_point().x());
  EXPECT_FLOAT_EQ(341.853799387, last_point.path_point().y());

  // middle
  const auto middle_point = obstacle->GetPointAtTime(3.7300);
  EXPECT_LE(3.68968892853, middle_point.relative_time());
  EXPECT_GE(3.89467164678, middle_point.relative_time());
  EXPECT_GE(139.091700103, middle_point.path_point().x());
  EXPECT_LE(135.817210975, middle_point.path_point().x());
  EXPECT_GE(349.875902219, middle_point.path_point().y());
  EXPECT_LE(349.549888973, middle_point.path_point().y());
}

TEST_F(ObstacleTest, PerceptionBoundingBox) {
  const auto* obstacle = indexed_obstacles_.Find("2156_0");
  ASSERT_TRUE(obstacle);
  const auto& box = obstacle->PerceptionBoundingBox();

  std::vector<common::math::Vec2d> corners;
  box.GetAllCorners(&corners);
  EXPECT_EQ(4, corners.size());
  EXPECT_FLOAT_EQ(3.832477, box.length());
  EXPECT_FLOAT_EQ(1.73200099013, box.width());
  EXPECT_FLOAT_EQ(76.684071405, box.center_x());
  EXPECT_FLOAT_EQ(350.481852505, box.center_y());
  EXPECT_FLOAT_EQ(0.00531211859358, box.heading());
}

TEST_F(ObstacleTest, GetBoundingBox) {
  const auto* obstacle = indexed_obstacles_.Find("2156_0");
  ASSERT_TRUE(obstacle);
  const auto& point = obstacle->Trajectory().trajectory_point(2);
  const auto& box = obstacle->GetBoundingBox(point);
  std::vector<common::math::Vec2d> corners;
  box.GetAllCorners(&corners);
  EXPECT_EQ(4, corners.size());
  EXPECT_FLOAT_EQ(3.832477, box.length());
  EXPECT_FLOAT_EQ(1.73200099013, box.width());
  EXPECT_FLOAT_EQ(83.2581699369, box.center_x());
  EXPECT_FLOAT_EQ(350.779556678, box.center_y());
  EXPECT_FLOAT_EQ(0.040689919, box.heading());
}

TEST_F(ObstacleTest, PerceptionPolygon) {
  const auto* obstacle = indexed_obstacles_.Find("2156_0");
  ASSERT_TRUE(obstacle);
  const auto& polygon = obstacle->PerceptionPolygon();

  const auto& points = polygon.points();
  EXPECT_EQ(16, points.size());
  EXPECT_FLOAT_EQ(74.766182, points[0].x());
  EXPECT_FLOAT_EQ(350.72986, points[0].y());
  EXPECT_FLOAT_EQ(74.783195, points[1].x());
  EXPECT_FLOAT_EQ(350.32602, points[1].y());
  EXPECT_FLOAT_EQ(74.770554, points[15].x());
  EXPECT_FLOAT_EQ(350.87857, points[15].y());
}

TEST_F(ObstacleTest, Trajectory) {
  const auto* obstacle = indexed_obstacles_.Find("2156_0");
  ASSERT_TRUE(obstacle);
  const auto& points = obstacle->Trajectory().trajectory_point();
  EXPECT_EQ(50, points.size());
}

TEST_F(ObstacleTest, Perception) {
  const auto* obstacle = indexed_obstacles_.Find("2156_0");
  ASSERT_TRUE(obstacle);
  const auto& perception_obstacle = obstacle->Perception();
  EXPECT_EQ(2156, perception_obstacle.id());
}

TEST(Obstacle, CreateStaticVirtualObstacle) {
  common::math::Box2d box({0, 0}, 0.0, 4.0, 2.0);
  std::unique_ptr<Obstacle> obstacle =
      Obstacle::CreateStaticVirtualObstacles("abc", box);
  EXPECT_EQ("abc", obstacle->Id());
  EXPECT_EQ(-314721735, obstacle->PerceptionId());
  EXPECT_TRUE(Obstacle::IsStaticObstacle(obstacle->Perception()));
  EXPECT_TRUE(Obstacle::IsVirtualObstacle(obstacle->Perception()));
  auto& perception_box = obstacle->PerceptionBoundingBox();
  EXPECT_FLOAT_EQ(0.0, perception_box.center().x());
  EXPECT_FLOAT_EQ(0.0, perception_box.center().y());
  EXPECT_FLOAT_EQ(4.0, perception_box.length());
  EXPECT_FLOAT_EQ(2.0, perception_box.width());
  EXPECT_FLOAT_EQ(0.0, perception_box.heading());
}

}  // namespace planning
}  // namespace apollo
