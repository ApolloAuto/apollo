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

#include "modules/planning/common/path_obstacle.h"

#include <memory>
#include <unordered_map>
#include <vector>

#include "gtest/gtest.h"
#include "modules/common/util/file.h"
#include "modules/common/util/util.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"

namespace apollo {
namespace planning {

TEST(IsLateralDecision, AllDecisions) {
  ObjectDecisionType decision_ignore;
  decision_ignore.mutable_ignore();
  EXPECT_TRUE(PathObstacle::IsLateralDecision(decision_ignore));

  ObjectDecisionType decision_overtake;
  decision_overtake.mutable_overtake();
  EXPECT_FALSE(PathObstacle::IsLateralDecision(decision_overtake));

  ObjectDecisionType decision_follow;
  decision_follow.mutable_follow();
  EXPECT_FALSE(PathObstacle::IsLateralDecision(decision_follow));

  ObjectDecisionType decision_yield;
  decision_yield.mutable_yield();
  EXPECT_FALSE(PathObstacle::IsLateralDecision(decision_yield));

  ObjectDecisionType decision_stop;
  decision_stop.mutable_stop();
  EXPECT_FALSE(PathObstacle::IsLateralDecision(decision_stop));

  ObjectDecisionType decision_nudge;
  decision_nudge.mutable_nudge();
  EXPECT_TRUE(PathObstacle::IsLateralDecision(decision_nudge));

  ObjectDecisionType decision_sidepass;
  decision_sidepass.mutable_sidepass();
  EXPECT_TRUE(PathObstacle::IsLateralDecision(decision_sidepass));
}

TEST(IsLongitudinalDecision, AllDecisions) {
  ObjectDecisionType decision_ignore;
  decision_ignore.mutable_ignore();
  EXPECT_TRUE(PathObstacle::IsLongitudinalDecision(decision_ignore));

  ObjectDecisionType decision_overtake;
  decision_overtake.mutable_overtake();
  EXPECT_TRUE(PathObstacle::IsLongitudinalDecision(decision_overtake));

  ObjectDecisionType decision_follow;
  decision_follow.mutable_follow();
  EXPECT_TRUE(PathObstacle::IsLongitudinalDecision(decision_follow));

  ObjectDecisionType decision_yield;
  decision_yield.mutable_yield();
  EXPECT_TRUE(PathObstacle::IsLongitudinalDecision(decision_yield));

  ObjectDecisionType decision_stop;
  decision_stop.mutable_stop();
  EXPECT_TRUE(PathObstacle::IsLongitudinalDecision(decision_stop));

  ObjectDecisionType decision_nudge;
  decision_nudge.mutable_nudge();
  EXPECT_FALSE(PathObstacle::IsLongitudinalDecision(decision_nudge));

  ObjectDecisionType decision_sidepass;
  decision_sidepass.mutable_sidepass();
  EXPECT_FALSE(PathObstacle::IsLongitudinalDecision(decision_sidepass));
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

  ObjectDecisionType decision_sidepass;
  decision_sidepass.mutable_sidepass();

  // vertical decision comparison
  EXPECT_TRUE(
      PathObstacle::MergeLongitudinalDecision(decision_stop, decision_ignore)
          .has_stop());
  EXPECT_TRUE(
      PathObstacle::MergeLongitudinalDecision(decision_stop, decision_overtake)
          .has_stop());
  EXPECT_TRUE(
      PathObstacle::MergeLongitudinalDecision(decision_stop, decision_follow)
          .has_stop());
  EXPECT_TRUE(
      PathObstacle::MergeLongitudinalDecision(decision_stop, decision_yield)
          .has_stop());

  EXPECT_TRUE(
      PathObstacle::MergeLongitudinalDecision(decision_yield, decision_ignore)
          .has_yield());
  EXPECT_TRUE(
      PathObstacle::MergeLongitudinalDecision(decision_yield, decision_overtake)
          .has_yield());
  EXPECT_TRUE(
      PathObstacle::MergeLongitudinalDecision(decision_yield, decision_follow)
          .has_yield());

  EXPECT_TRUE(
      PathObstacle::MergeLongitudinalDecision(decision_follow, decision_ignore)
          .has_follow());
  EXPECT_TRUE(PathObstacle::MergeLongitudinalDecision(decision_follow,
                                                      decision_overtake)
                  .has_follow());

  EXPECT_TRUE(PathObstacle::MergeLongitudinalDecision(decision_overtake,
                                                      decision_ignore)
                  .has_overtake());

  EXPECT_TRUE(PathObstacle::MergeLongitudinalDecision(decision_ignore,
                                                      decision_overtake)
                  .has_overtake());
  EXPECT_TRUE(
      PathObstacle::MergeLongitudinalDecision(decision_ignore, decision_follow)
          .has_follow());
  EXPECT_TRUE(
      PathObstacle::MergeLongitudinalDecision(decision_ignore, decision_yield)
          .has_yield());
  EXPECT_TRUE(
      PathObstacle::MergeLongitudinalDecision(decision_ignore, decision_stop)
          .has_stop());

  EXPECT_TRUE(PathObstacle::MergeLongitudinalDecision(decision_overtake,
                                                      decision_follow)
                  .has_follow());
  EXPECT_TRUE(
      PathObstacle::MergeLongitudinalDecision(decision_overtake, decision_yield)
          .has_yield());
  EXPECT_TRUE(
      PathObstacle::MergeLongitudinalDecision(decision_overtake, decision_stop)
          .has_stop());

  EXPECT_TRUE(
      PathObstacle::MergeLongitudinalDecision(decision_follow, decision_yield)
          .has_yield());
  EXPECT_TRUE(
      PathObstacle::MergeLongitudinalDecision(decision_follow, decision_stop)
          .has_stop());

  EXPECT_TRUE(
      PathObstacle::MergeLongitudinalDecision(decision_yield, decision_stop)
          .has_stop());

  EXPECT_TRUE(
      PathObstacle::MergeLongitudinalDecision(decision_ignore, decision_ignore)
          .has_ignore());

  ObjectDecisionType decision_overtake1;
  decision_overtake1.mutable_overtake()->set_distance_s(1);
  ObjectDecisionType decision_overtake2;
  decision_overtake2.mutable_overtake()->set_distance_s(2);
  EXPECT_EQ(2, PathObstacle::MergeLongitudinalDecision(decision_overtake1,
                                                       decision_overtake2)
                   .overtake()
                   .distance_s());

  ObjectDecisionType decision_follow1;
  decision_follow1.mutable_follow()->set_distance_s(-1);
  ObjectDecisionType decision_follow2;
  decision_follow2.mutable_follow()->set_distance_s(-2);
  EXPECT_EQ(-2, PathObstacle::MergeLongitudinalDecision(decision_follow1,
                                                        decision_follow2)
                    .follow()
                    .distance_s());

  ObjectDecisionType decision_yield1;
  decision_yield1.mutable_yield()->set_distance_s(-1);
  ObjectDecisionType decision_yield2;
  decision_yield2.mutable_yield()->set_distance_s(-2);
  EXPECT_EQ(-2, PathObstacle::MergeLongitudinalDecision(decision_yield1,
                                                        decision_yield2)
                    .yield()
                    .distance_s());

  ObjectDecisionType decision_stop1;
  decision_stop1.mutable_stop()->set_distance_s(-1);
  ObjectDecisionType decision_stop2;
  decision_stop2.mutable_stop()->set_distance_s(-2);
  EXPECT_EQ(-2, PathObstacle::MergeLongitudinalDecision(decision_stop1,
                                                        decision_stop2)
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

  ObjectDecisionType decision_sidepass;
  decision_sidepass.mutable_sidepass();

  EXPECT_TRUE(
      PathObstacle::MergeLateralDecision(decision_nudge, decision_ignore)
          .has_nudge());

  EXPECT_TRUE(
      PathObstacle::MergeLateralDecision(decision_ignore, decision_nudge)
          .has_nudge());

  ObjectDecisionType decision_nudge2;
  decision_nudge2.mutable_nudge()->set_type(ObjectNudge::LEFT_NUDGE);
  EXPECT_TRUE(
      PathObstacle::MergeLateralDecision(decision_nudge, decision_nudge2)
          .has_nudge());
  decision_nudge2.mutable_nudge()->set_type(ObjectNudge::RIGHT_NUDGE);
}

TEST(PathObstacleTest, add_decision_test) {
  // init state
  {
    PathObstacle path_obstacle;
    EXPECT_FALSE(path_obstacle.HasLateralDecision());
    EXPECT_FALSE(path_obstacle.HasLongitudinalDecision());
  }

  // Ignore
  {
    PathObstacle path_obstacle;
    ObjectDecisionType decision;
    decision.mutable_ignore();
    path_obstacle.AddLongitudinalDecision("test_ignore", decision);
    EXPECT_FALSE(path_obstacle.HasLateralDecision());
    EXPECT_TRUE(path_obstacle.HasLongitudinalDecision());
    EXPECT_FALSE(path_obstacle.LateralDecision().has_ignore());
    EXPECT_TRUE(path_obstacle.LongitudinalDecision().has_ignore());
  }

  // stop and ignore
  {
    PathObstacle path_obstacle;
    ObjectDecisionType decision;

    decision.mutable_stop();
    path_obstacle.AddLongitudinalDecision("test_stop", decision);

    EXPECT_FALSE(path_obstacle.HasLateralDecision());
    EXPECT_TRUE(path_obstacle.HasLongitudinalDecision());
    EXPECT_TRUE(path_obstacle.LongitudinalDecision().has_stop());

    decision.mutable_ignore();
    path_obstacle.AddLongitudinalDecision("test_ignore", decision);
    EXPECT_FALSE(path_obstacle.HasLateralDecision());
    EXPECT_TRUE(path_obstacle.HasLongitudinalDecision());
    EXPECT_FALSE(path_obstacle.LateralDecision().has_ignore());
    EXPECT_TRUE(path_obstacle.LongitudinalDecision().has_stop());
  }

  // left nudge and ignore
  {
    PathObstacle path_obstacle;
    ObjectDecisionType decision;

    decision.mutable_nudge()->set_type(ObjectNudge::LEFT_NUDGE);
    path_obstacle.AddLateralDecision("test_nudge", decision);

    EXPECT_TRUE(path_obstacle.HasLateralDecision());
    EXPECT_FALSE(path_obstacle.HasLongitudinalDecision());
    EXPECT_TRUE(path_obstacle.LateralDecision().has_nudge());

    decision.mutable_ignore();
    path_obstacle.AddLateralDecision("test_ignore", decision);
    EXPECT_TRUE(path_obstacle.HasLateralDecision());
    EXPECT_FALSE(path_obstacle.HasLongitudinalDecision());
    EXPECT_TRUE(path_obstacle.LateralDecision().has_nudge());
    EXPECT_FALSE(path_obstacle.LongitudinalDecision().has_ignore());
  }

  // right nudge and ignore
  {
    PathObstacle path_obstacle;
    ObjectDecisionType decision;

    decision.mutable_nudge()->set_type(ObjectNudge::RIGHT_NUDGE);
    path_obstacle.AddLateralDecision("test_nudge", decision);

    EXPECT_TRUE(path_obstacle.HasLateralDecision());
    EXPECT_FALSE(path_obstacle.HasLongitudinalDecision());
    EXPECT_TRUE(path_obstacle.LateralDecision().has_nudge());

    decision.mutable_ignore();
    path_obstacle.AddLateralDecision("test_ignore", decision);
    EXPECT_TRUE(path_obstacle.HasLateralDecision());
    EXPECT_FALSE(path_obstacle.HasLongitudinalDecision());
    EXPECT_TRUE(path_obstacle.LateralDecision().has_nudge());
    EXPECT_FALSE(path_obstacle.LongitudinalDecision().has_ignore());
  }

  // left nudge and right nudge
  {
    PathObstacle path_obstacle;
    ObjectDecisionType decision;

    decision.mutable_nudge()->set_type(ObjectNudge::LEFT_NUDGE);
    path_obstacle.AddLateralDecision("test_left_nudge", decision);

    EXPECT_TRUE(path_obstacle.HasLateralDecision());
    EXPECT_FALSE(path_obstacle.HasLongitudinalDecision());
    EXPECT_TRUE(path_obstacle.LateralDecision().has_nudge());
  }

  // overtake and ignore
  {
    PathObstacle path_obstacle;
    ObjectDecisionType decision;

    decision.mutable_overtake();
    path_obstacle.AddLongitudinalDecision("test_overtake", decision);

    EXPECT_FALSE(path_obstacle.HasLateralDecision());
    EXPECT_TRUE(path_obstacle.HasLongitudinalDecision());
    EXPECT_TRUE(path_obstacle.LongitudinalDecision().has_overtake());

    decision.mutable_ignore();
    path_obstacle.AddLongitudinalDecision("test_ignore", decision);
    EXPECT_FALSE(path_obstacle.HasLateralDecision());
    EXPECT_TRUE(path_obstacle.HasLongitudinalDecision());
    EXPECT_FALSE(path_obstacle.LateralDecision().has_ignore());
    EXPECT_TRUE(path_obstacle.LongitudinalDecision().has_overtake());
  }

  // yield and ignore
  {
    PathObstacle path_obstacle;
    ObjectDecisionType decision;

    decision.mutable_yield();
    path_obstacle.AddLongitudinalDecision("test_yield", decision);

    EXPECT_FALSE(path_obstacle.HasLateralDecision());
    EXPECT_TRUE(path_obstacle.HasLongitudinalDecision());
    EXPECT_TRUE(path_obstacle.LongitudinalDecision().has_yield());

    decision.mutable_ignore();
    path_obstacle.AddLongitudinalDecision("test_ignore", decision);
    EXPECT_FALSE(path_obstacle.HasLateralDecision());
    EXPECT_TRUE(path_obstacle.HasLongitudinalDecision());
    EXPECT_FALSE(path_obstacle.LateralDecision().has_ignore());
    EXPECT_TRUE(path_obstacle.LongitudinalDecision().has_yield());
  }

  // stop and nudge
  {
    PathObstacle path_obstacle;
    ObjectDecisionType decision;

    decision.mutable_stop();
    path_obstacle.AddLongitudinalDecision("test_stop", decision);

    EXPECT_FALSE(path_obstacle.HasLateralDecision());
    EXPECT_TRUE(path_obstacle.HasLongitudinalDecision());
    EXPECT_TRUE(path_obstacle.LongitudinalDecision().has_stop());

    decision.mutable_nudge();
    path_obstacle.AddLateralDecision("test_nudge", decision);
    EXPECT_TRUE(path_obstacle.HasLateralDecision());
    EXPECT_TRUE(path_obstacle.HasLongitudinalDecision());
    EXPECT_TRUE(path_obstacle.LateralDecision().has_nudge());
    EXPECT_TRUE(path_obstacle.LongitudinalDecision().has_stop());
  }
}

}  // namespace planning
}  // namespace apollo
