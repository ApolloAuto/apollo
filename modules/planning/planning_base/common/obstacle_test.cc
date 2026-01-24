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

#include "modules/planning/planning_base/common/obstacle.h"

#include "gtest/gtest.h"

#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "modules/common_msgs/prediction_msgs/prediction_obstacle.pb.h"

#include "cyber/common/file.h"
#include "modules/common/util/util.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::perception::PerceptionObstacle;

TEST(Obstacle, IsValidPerceptionObstacle) {
  PerceptionObstacle perception_obstacle;
  EXPECT_FALSE(Obstacle::IsValidPerceptionObstacle(perception_obstacle));

  perception_obstacle.mutable_position()->set_x(2.5);
  perception_obstacle.mutable_position()->set_y(0.5);
  perception_obstacle.mutable_position()->set_z(0.5);

  perception_obstacle.mutable_velocity()->set_x(2.5);
  perception_obstacle.mutable_velocity()->set_y(0.5);
  perception_obstacle.mutable_velocity()->set_z(0.5);
  EXPECT_FALSE(Obstacle::IsValidPerceptionObstacle(perception_obstacle));

  perception_obstacle.set_length(0.1);
  EXPECT_FALSE(Obstacle::IsValidPerceptionObstacle(perception_obstacle));

  perception_obstacle.set_width(0.1);
  EXPECT_FALSE(Obstacle::IsValidPerceptionObstacle(perception_obstacle));

  perception_obstacle.set_height(0.1);
  EXPECT_TRUE(Obstacle::IsValidPerceptionObstacle(perception_obstacle));
}

class ObstacleTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    prediction::PredictionObstacles prediction_obstacles;
    ASSERT_TRUE(cyber::common::GetProtoFromFile(
        "/apollo/modules/planning/planning_base/testdata/common/"
        "sample_prediction.pb.txt",
        &prediction_obstacles));
    auto obstacles = Obstacle::CreateObstacles(prediction_obstacles);
    ASSERT_EQ(6, obstacles.size());
    for (auto& obstacle : obstacles) {
      const auto id = obstacle->Id();
      indexed_obstacles_.Add(id, *obstacle);
    }
  }

 protected:
  IndexedObstacles indexed_obstacles_;
};

TEST_F(ObstacleTest, CreateObstacles) {
  ASSERT_EQ(6, indexed_obstacles_.Items().size());
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
  EXPECT_DOUBLE_EQ(0.0, first_point.relative_time());
  EXPECT_DOUBLE_EQ(76.684071405, first_point.path_point().x());
  EXPECT_DOUBLE_EQ(350.481852505, first_point.path_point().y());

  // last
  const auto last_point = obstacle->GetPointAtTime(10.04415320);
  EXPECT_DOUBLE_EQ(10.0441531943, last_point.relative_time());
  EXPECT_DOUBLE_EQ(186.259371951, last_point.path_point().x());
  EXPECT_DOUBLE_EQ(341.853799387, last_point.path_point().y());

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
  EXPECT_DOUBLE_EQ(3.8324769804600001, box.length());
  EXPECT_DOUBLE_EQ(1.73200099013, box.width());
  EXPECT_DOUBLE_EQ(76.684071405, box.center_x());
  EXPECT_DOUBLE_EQ(350.481852505, box.center_y());
  EXPECT_DOUBLE_EQ(0.00531211859358, box.heading());
}

TEST_F(ObstacleTest, GetBoundingBox) {
  const auto* obstacle = indexed_obstacles_.Find("2156_0");
  ASSERT_TRUE(obstacle);
  const auto& point = obstacle->Trajectory().trajectory_point(2);
  const auto& box = obstacle->GetBoundingBox(point);
  std::vector<common::math::Vec2d> corners;
  box.GetAllCorners(&corners);
  EXPECT_EQ(4, corners.size());
  EXPECT_DOUBLE_EQ(3.8324769804600001, box.length());
  EXPECT_DOUBLE_EQ(1.73200099013, box.width());
  EXPECT_DOUBLE_EQ(83.2581699369, box.center_x());
  EXPECT_DOUBLE_EQ(350.779556678, box.center_y());
  EXPECT_DOUBLE_EQ(0.040689919196199999, box.heading());
}

TEST_F(ObstacleTest, PerceptionPolygon) {
  const auto* obstacle = indexed_obstacles_.Find("2156_0");
  ASSERT_TRUE(obstacle);
  const auto& polygon = obstacle->PerceptionPolygon();

  const auto& points = polygon.points();
  EXPECT_EQ(16, points.size());
  EXPECT_DOUBLE_EQ(74.766181894499994, points[0].x());
  EXPECT_DOUBLE_EQ(350.72985818299998, points[0].y());
  EXPECT_DOUBLE_EQ(74.783198034400002, points[1].x());
  EXPECT_DOUBLE_EQ(350.32601568799998, points[1].y());
  EXPECT_DOUBLE_EQ(74.770555729799995, points[15].x());
  EXPECT_DOUBLE_EQ(350.878567223, points[15].y());
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
  EXPECT_TRUE(obstacle->IsVirtual());
  auto& perception_box = obstacle->PerceptionBoundingBox();
  EXPECT_DOUBLE_EQ(0.0, perception_box.center().x());
  EXPECT_DOUBLE_EQ(0.0, perception_box.center().y());
  EXPECT_DOUBLE_EQ(4.0, perception_box.length());
  EXPECT_DOUBLE_EQ(2.0, perception_box.width());
  EXPECT_DOUBLE_EQ(0.0, perception_box.heading());
}

TEST(IsLateralDecision, AllDecisions) {
  ObjectDecisionType decision_ignore;
  decision_ignore.mutable_ignore();
  EXPECT_TRUE(Obstacle::IsLateralDecision(decision_ignore));

  ObjectDecisionType decision_overtake;
  decision_overtake.mutable_overtake();
  EXPECT_FALSE(Obstacle::IsLateralDecision(decision_overtake));

  ObjectDecisionType decision_follow;
  decision_follow.mutable_follow();
  EXPECT_FALSE(Obstacle::IsLateralDecision(decision_follow));

  ObjectDecisionType decision_yield;
  decision_yield.mutable_yield();
  EXPECT_FALSE(Obstacle::IsLateralDecision(decision_yield));

  ObjectDecisionType decision_stop;
  decision_stop.mutable_stop();
  EXPECT_FALSE(Obstacle::IsLateralDecision(decision_stop));

  ObjectDecisionType decision_nudge;
  decision_nudge.mutable_nudge();
  EXPECT_TRUE(Obstacle::IsLateralDecision(decision_nudge));
}

TEST(IsLongitudinalDecision, AllDecisions) {
  ObjectDecisionType decision_ignore;
  decision_ignore.mutable_ignore();
  EXPECT_TRUE(Obstacle::IsLongitudinalDecision(decision_ignore));

  ObjectDecisionType decision_overtake;
  decision_overtake.mutable_overtake();
  EXPECT_TRUE(Obstacle::IsLongitudinalDecision(decision_overtake));

  ObjectDecisionType decision_follow;
  decision_follow.mutable_follow();
  EXPECT_TRUE(Obstacle::IsLongitudinalDecision(decision_follow));

  ObjectDecisionType decision_yield;
  decision_yield.mutable_yield();
  EXPECT_TRUE(Obstacle::IsLongitudinalDecision(decision_yield));

  ObjectDecisionType decision_stop;
  decision_stop.mutable_stop();
  EXPECT_TRUE(Obstacle::IsLongitudinalDecision(decision_stop));

  ObjectDecisionType decision_nudge;
  decision_nudge.mutable_nudge();
  EXPECT_FALSE(Obstacle::IsLongitudinalDecision(decision_nudge));
}

TEST(MergeLongitudinalDecision, AllDecisions) {
  ObjectDecisionType decision_ignore;
  decision_ignore.mutable_ignore();

  ObjectDecisionType decision_overtake;
  decision_overtake.mutable_overtake();

  ObjectDecisionType decision_follow;
  decision_follow.mutable_follow();

  ObjectDecisionType decision_yield;
  decision_yield.mutable_yield();

  ObjectDecisionType decision_stop;
  decision_stop.mutable_stop();

  ObjectDecisionType decision_nudge;
  decision_nudge.mutable_nudge();

  // vertical decision comparison
  EXPECT_TRUE(
      Obstacle::MergeLongitudinalDecision(decision_stop, decision_ignore)
          .has_stop());
  EXPECT_TRUE(
      Obstacle::MergeLongitudinalDecision(decision_stop, decision_overtake)
          .has_stop());
  EXPECT_TRUE(
      Obstacle::MergeLongitudinalDecision(decision_stop, decision_follow)
          .has_stop());
  EXPECT_TRUE(Obstacle::MergeLongitudinalDecision(decision_stop, decision_yield)
                  .has_stop());

  EXPECT_TRUE(
      Obstacle::MergeLongitudinalDecision(decision_yield, decision_ignore)
          .has_yield());
  EXPECT_TRUE(
      Obstacle::MergeLongitudinalDecision(decision_yield, decision_overtake)
          .has_yield());
  EXPECT_TRUE(
      Obstacle::MergeLongitudinalDecision(decision_yield, decision_follow)
          .has_yield());

  EXPECT_TRUE(
      Obstacle::MergeLongitudinalDecision(decision_follow, decision_ignore)
          .has_follow());
  EXPECT_TRUE(
      Obstacle::MergeLongitudinalDecision(decision_follow, decision_overtake)
          .has_follow());

  EXPECT_TRUE(
      Obstacle::MergeLongitudinalDecision(decision_overtake, decision_ignore)
          .has_overtake());

  EXPECT_TRUE(
      Obstacle::MergeLongitudinalDecision(decision_ignore, decision_overtake)
          .has_overtake());
  EXPECT_TRUE(
      Obstacle::MergeLongitudinalDecision(decision_ignore, decision_follow)
          .has_follow());
  EXPECT_TRUE(
      Obstacle::MergeLongitudinalDecision(decision_ignore, decision_yield)
          .has_yield());
  EXPECT_TRUE(
      Obstacle::MergeLongitudinalDecision(decision_ignore, decision_stop)
          .has_stop());

  EXPECT_TRUE(
      Obstacle::MergeLongitudinalDecision(decision_overtake, decision_follow)
          .has_follow());
  EXPECT_TRUE(
      Obstacle::MergeLongitudinalDecision(decision_overtake, decision_yield)
          .has_yield());
  EXPECT_TRUE(
      Obstacle::MergeLongitudinalDecision(decision_overtake, decision_stop)
          .has_stop());

  EXPECT_TRUE(
      Obstacle::MergeLongitudinalDecision(decision_follow, decision_yield)
          .has_yield());
  EXPECT_TRUE(
      Obstacle::MergeLongitudinalDecision(decision_follow, decision_stop)
          .has_stop());

  EXPECT_TRUE(Obstacle::MergeLongitudinalDecision(decision_yield, decision_stop)
                  .has_stop());

  EXPECT_TRUE(
      Obstacle::MergeLongitudinalDecision(decision_ignore, decision_ignore)
          .has_ignore());

  ObjectDecisionType decision_overtake1;
  decision_overtake1.mutable_overtake()->set_distance_s(1);
  ObjectDecisionType decision_overtake2;
  decision_overtake2.mutable_overtake()->set_distance_s(2);
  EXPECT_EQ(2, Obstacle::MergeLongitudinalDecision(decision_overtake1,
                                                   decision_overtake2)
                   .overtake()
                   .distance_s());

  ObjectDecisionType decision_follow1;
  decision_follow1.mutable_follow()->set_distance_s(-1);
  ObjectDecisionType decision_follow2;
  decision_follow2.mutable_follow()->set_distance_s(-2);
  EXPECT_EQ(-2, Obstacle::MergeLongitudinalDecision(decision_follow1,
                                                    decision_follow2)
                    .follow()
                    .distance_s());

  ObjectDecisionType decision_yield1;
  decision_yield1.mutable_yield()->set_distance_s(-1);
  ObjectDecisionType decision_yield2;
  decision_yield2.mutable_yield()->set_distance_s(-2);
  EXPECT_EQ(
      -2, Obstacle::MergeLongitudinalDecision(decision_yield1, decision_yield2)
              .yield()
              .distance_s());

  ObjectDecisionType decision_stop1;
  decision_stop1.mutable_stop()->set_distance_s(-1);
  ObjectDecisionType decision_stop2;
  decision_stop2.mutable_stop()->set_distance_s(-2);
  EXPECT_EQ(-2,
            Obstacle::MergeLongitudinalDecision(decision_stop1, decision_stop2)
                .stop()
                .distance_s());
}

TEST(MergeLateralDecision, AllDecisions) {
  ObjectDecisionType decision_ignore;
  decision_ignore.mutable_ignore();

  ObjectDecisionType decision_overtake;
  decision_overtake.mutable_overtake();

  ObjectDecisionType decision_follow;
  decision_follow.mutable_follow();

  ObjectDecisionType decision_yield;
  decision_yield.mutable_yield();

  ObjectDecisionType decision_stop;
  decision_stop.mutable_stop();

  ObjectDecisionType decision_nudge;
  decision_nudge.mutable_nudge()->set_type(ObjectNudge::LEFT_NUDGE);

  EXPECT_TRUE(Obstacle::MergeLateralDecision(decision_nudge, decision_ignore)
                  .has_nudge());

  EXPECT_TRUE(Obstacle::MergeLateralDecision(decision_ignore, decision_nudge)
                  .has_nudge());

  ObjectDecisionType decision_nudge2;
  decision_nudge2.mutable_nudge()->set_type(ObjectNudge::LEFT_NUDGE);
  EXPECT_TRUE(Obstacle::MergeLateralDecision(decision_nudge, decision_nudge2)
                  .has_nudge());
  decision_nudge2.mutable_nudge()->set_type(ObjectNudge::RIGHT_NUDGE);
}

TEST(ObstacleMergeTest, add_decision_test) {
  // init state
  {
    Obstacle obstacle;
    EXPECT_FALSE(obstacle.HasLateralDecision());
    EXPECT_FALSE(obstacle.HasLongitudinalDecision());
  }

  // Ignore
  {
    Obstacle obstacle;
    ObjectDecisionType decision;
    decision.mutable_ignore();
    obstacle.AddLongitudinalDecision("test_ignore", decision);
    EXPECT_FALSE(obstacle.HasLateralDecision());
    EXPECT_TRUE(obstacle.HasLongitudinalDecision());
    EXPECT_FALSE(obstacle.LateralDecision().has_ignore());
    EXPECT_TRUE(obstacle.LongitudinalDecision().has_ignore());
  }

  // stop and ignore
  {
    Obstacle obstacle;
    ObjectDecisionType decision;

    decision.mutable_stop();
    obstacle.AddLongitudinalDecision("test_stop", decision);

    EXPECT_FALSE(obstacle.HasLateralDecision());
    EXPECT_TRUE(obstacle.HasLongitudinalDecision());
    EXPECT_TRUE(obstacle.LongitudinalDecision().has_stop());

    decision.mutable_ignore();
    obstacle.AddLongitudinalDecision("test_ignore", decision);
    EXPECT_FALSE(obstacle.HasLateralDecision());
    EXPECT_TRUE(obstacle.HasLongitudinalDecision());
    EXPECT_FALSE(obstacle.LateralDecision().has_ignore());
    EXPECT_TRUE(obstacle.LongitudinalDecision().has_stop());
  }

  // left nudge and ignore
  {
    Obstacle obstacle;
    ObjectDecisionType decision;

    decision.mutable_nudge()->set_type(ObjectNudge::LEFT_NUDGE);
    obstacle.AddLateralDecision("test_nudge", decision);

    EXPECT_TRUE(obstacle.HasLateralDecision());
    EXPECT_FALSE(obstacle.HasLongitudinalDecision());
    EXPECT_TRUE(obstacle.LateralDecision().has_nudge());

    decision.mutable_ignore();
    obstacle.AddLateralDecision("test_ignore", decision);
    EXPECT_TRUE(obstacle.HasLateralDecision());
    EXPECT_FALSE(obstacle.HasLongitudinalDecision());
    EXPECT_TRUE(obstacle.LateralDecision().has_nudge());
    EXPECT_FALSE(obstacle.LongitudinalDecision().has_ignore());
  }

  // right nudge and ignore
  {
    Obstacle obstacle;
    ObjectDecisionType decision;

    decision.mutable_nudge()->set_type(ObjectNudge::RIGHT_NUDGE);
    obstacle.AddLateralDecision("test_nudge", decision);

    EXPECT_TRUE(obstacle.HasLateralDecision());
    EXPECT_FALSE(obstacle.HasLongitudinalDecision());
    EXPECT_TRUE(obstacle.LateralDecision().has_nudge());

    decision.mutable_ignore();
    obstacle.AddLateralDecision("test_ignore", decision);
    EXPECT_TRUE(obstacle.HasLateralDecision());
    EXPECT_FALSE(obstacle.HasLongitudinalDecision());
    EXPECT_TRUE(obstacle.LateralDecision().has_nudge());
    EXPECT_FALSE(obstacle.LongitudinalDecision().has_ignore());
  }

  // left nudge and right nudge
  {
    Obstacle obstacle;
    ObjectDecisionType decision;

    decision.mutable_nudge()->set_type(ObjectNudge::LEFT_NUDGE);
    obstacle.AddLateralDecision("test_left_nudge", decision);

    EXPECT_TRUE(obstacle.HasLateralDecision());
    EXPECT_FALSE(obstacle.HasLongitudinalDecision());
    EXPECT_TRUE(obstacle.LateralDecision().has_nudge());
  }

  // overtake and ignore
  {
    Obstacle obstacle;
    ObjectDecisionType decision;

    decision.mutable_overtake();
    obstacle.AddLongitudinalDecision("test_overtake", decision);

    EXPECT_FALSE(obstacle.HasLateralDecision());
    EXPECT_TRUE(obstacle.HasLongitudinalDecision());
    EXPECT_TRUE(obstacle.LongitudinalDecision().has_overtake());

    decision.mutable_ignore();
    obstacle.AddLongitudinalDecision("test_ignore", decision);
    EXPECT_FALSE(obstacle.HasLateralDecision());
    EXPECT_TRUE(obstacle.HasLongitudinalDecision());
    EXPECT_FALSE(obstacle.LateralDecision().has_ignore());
    EXPECT_TRUE(obstacle.LongitudinalDecision().has_overtake());
  }

  // follow and ignore
  {
    Obstacle obstacle;
    ObjectDecisionType decision;

    decision.mutable_follow();
    obstacle.AddLongitudinalDecision("test_follow", decision);

    EXPECT_FALSE(obstacle.HasLateralDecision());
    EXPECT_TRUE(obstacle.HasLongitudinalDecision());
    EXPECT_TRUE(obstacle.LongitudinalDecision().has_follow());

    decision.mutable_ignore();
    obstacle.AddLongitudinalDecision("test_ignore", decision);
    EXPECT_FALSE(obstacle.HasLateralDecision());
    EXPECT_TRUE(obstacle.HasLongitudinalDecision());
    EXPECT_FALSE(obstacle.LateralDecision().has_ignore());
    EXPECT_TRUE(obstacle.LongitudinalDecision().has_follow());
  }

  // yield and ignore
  {
    Obstacle obstacle;
    ObjectDecisionType decision;

    decision.mutable_yield();
    obstacle.AddLongitudinalDecision("test_yield", decision);

    EXPECT_FALSE(obstacle.HasLateralDecision());
    EXPECT_TRUE(obstacle.HasLongitudinalDecision());
    EXPECT_TRUE(obstacle.LongitudinalDecision().has_yield());

    decision.mutable_ignore();
    obstacle.AddLongitudinalDecision("test_ignore", decision);
    EXPECT_FALSE(obstacle.HasLateralDecision());
    EXPECT_TRUE(obstacle.HasLongitudinalDecision());
    EXPECT_FALSE(obstacle.LateralDecision().has_ignore());
    EXPECT_TRUE(obstacle.LongitudinalDecision().has_yield());
  }

  // stop and nudge
  {
    Obstacle obstacle;
    ObjectDecisionType decision;

    decision.mutable_stop();
    obstacle.AddLongitudinalDecision("test_stop", decision);

    EXPECT_FALSE(obstacle.HasLateralDecision());
    EXPECT_TRUE(obstacle.HasLongitudinalDecision());
    EXPECT_TRUE(obstacle.LongitudinalDecision().has_stop());

    decision.mutable_nudge();
    obstacle.AddLateralDecision("test_nudge", decision);
    EXPECT_TRUE(obstacle.HasLateralDecision());
    EXPECT_TRUE(obstacle.HasLongitudinalDecision());
    EXPECT_TRUE(obstacle.LateralDecision().has_nudge());
    EXPECT_TRUE(obstacle.LongitudinalDecision().has_stop());
  }

  // stop and yield
  {
    Obstacle obstacle;
    ObjectDecisionType decision;

    decision.mutable_stop();
    obstacle.AddLongitudinalDecision("test_stop", decision);

    EXPECT_FALSE(obstacle.HasLateralDecision());
    EXPECT_TRUE(obstacle.HasLongitudinalDecision());
    EXPECT_TRUE(obstacle.LongitudinalDecision().has_stop());

    decision.mutable_yield();
    obstacle.AddLongitudinalDecision("test_yield", decision);
    EXPECT_FALSE(obstacle.HasLateralDecision());
    EXPECT_TRUE(obstacle.HasLongitudinalDecision());
    EXPECT_TRUE(obstacle.LongitudinalDecision().has_stop());
  }
}

}  // namespace planning
}  // namespace apollo
