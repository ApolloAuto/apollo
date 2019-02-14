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

#include "modules/map/pnc_map/pnc_map.h"

#include "gflags/gflags.h"
#include "gtest/gtest.h"

#include "cyber/common/file.h"
#include "modules/common/util/string_util.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/routing/proto/routing.pb.h"

DECLARE_double(min_length_for_lane_change);

namespace apollo {
namespace hdmap {

DEFINE_string(test_map_file,
              "modules/map/data/sunnyvale_loop/base_map_test.bin",
              "The test map file");
DEFINE_string(
    test_routing_file,
    "modules/map/pnc_map/testdata/sample_sunnyvale_loop_routing.pb.txt",
    "The test map file");

class PncMapTest : public ::testing::Test {
 public:
  static void SetUpTestCase() {
    AINFO << "map file: " << FLAGS_test_map_file;
    if (hdmap_.LoadMapFromFile(FLAGS_test_map_file) != 0) {
      AERROR << "Failed to load map: " << FLAGS_test_map_file;
      CHECK(false);
    }
    pnc_map_.reset(new PncMap(&hdmap_));
    if (!cyber::common::GetProtoFromFile(FLAGS_test_routing_file, &routing_)) {
      AERROR << "Failed to load routing: " << FLAGS_test_routing_file;
      CHECK(false);
    }
    pnc_map_->UpdateRoutingResponse(routing_);
  }

  static double RouteLength(const RouteSegments& segments) {
    double s = 0.0;
    for (const auto& seg : segments) {
      s += seg.end_s - seg.start_s;
    }
    return s;
  }

  static routing::RoutingResponse routing_;
  static std::unique_ptr<PncMap> pnc_map_;
  static hdmap::HDMap hdmap_;
};

std::unique_ptr<PncMap> PncMapTest::pnc_map_;
hdmap::HDMap PncMapTest::hdmap_;
routing::RoutingResponse PncMapTest::routing_;

TEST_F(PncMapTest, UpdateRouting) {
  pnc_map_->routing_.clear_header();
  EXPECT_TRUE(pnc_map_->IsNewRouting(routing_));
  EXPECT_TRUE(pnc_map_->UpdateRoutingResponse(routing_));
  EXPECT_FALSE(pnc_map_->IsNewRouting(routing_));
}

TEST_F(PncMapTest, GetNearestPointFromRouting) {
  LaneWaypoint waypoint;
  common::VehicleState state;
  state.set_x(587174.662136);
  state.set_y(4140933.06302);  // the lane heading at this spot is about 0.9 PI
  state.set_heading(0.0);
  EXPECT_FALSE(pnc_map_->GetNearestPointFromRouting(state, &waypoint));
  state.set_heading(M_PI);
  EXPECT_TRUE(pnc_map_->GetNearestPointFromRouting(state, &waypoint));
  ASSERT_TRUE(waypoint.lane != nullptr);
  EXPECT_EQ("9_1_-1", waypoint.lane->id().id());
  EXPECT_FLOAT_EQ(60.757099, waypoint.s);
}
TEST_F(PncMapTest, UpdateWaypointIndex) {
  auto lane = hdmap_.GetLaneById(hdmap::MakeMapId("9_1_-1"));
  ASSERT_TRUE(lane);
  LaneWaypoint waypoint(lane, 60.757099);
  auto result = pnc_map_->GetWaypointIndex(waypoint);
  EXPECT_EQ(14, result);
}

TEST_F(PncMapTest, GetRouteSegments_NoChangeLane) {
  auto lane = hdmap_.GetLaneById(hdmap::MakeMapId("9_1_-2"));
  ASSERT_TRUE(lane);
  auto point = lane->GetSmoothPoint(0);
  common::VehicleState state;
  state.set_x(point.x());
  state.set_y(point.y());
  state.set_z(point.y());
  state.set_heading(M_PI);
  std::list<RouteSegments> segments;
  ASSERT_TRUE(pnc_map_->GetRouteSegments(state, 10, 30, &segments));
  // first time on this passage, should not immediately change lane
  ASSERT_EQ(2, segments.size());
  EXPECT_NEAR(40, RouteLength(segments.front()), 1e-4);
  EXPECT_NEAR(40, RouteLength(segments.back()), 1e-4);
  EXPECT_EQ(routing::LEFT, segments.front().NextAction());
  EXPECT_TRUE(segments.front().IsOnSegment());
  EXPECT_EQ(routing::RIGHT, segments.back().NextAction());
  EXPECT_FALSE(segments.back().IsOnSegment());
}

TEST_F(PncMapTest, UpdateNextRoutingWaypointIndex) {
  pnc_map_->next_routing_waypoint_index_ = 0;
  pnc_map_->adc_waypoint_.s = 0;
  pnc_map_->UpdateNextRoutingWaypointIndex(0);
  EXPECT_EQ(0, pnc_map_->next_routing_waypoint_index_);

  pnc_map_->adc_waypoint_.s = 50;
  pnc_map_->UpdateNextRoutingWaypointIndex(0);
  EXPECT_EQ(1, pnc_map_->next_routing_waypoint_index_);

  pnc_map_->adc_waypoint_.s = 63.6,
  pnc_map_->UpdateNextRoutingWaypointIndex(18);
  EXPECT_EQ(3, pnc_map_->next_routing_waypoint_index_);

  pnc_map_->adc_waypoint_.s = 63.8,
  pnc_map_->UpdateNextRoutingWaypointIndex(17);
  EXPECT_EQ(3, pnc_map_->next_routing_waypoint_index_);

  pnc_map_->adc_waypoint_.s = 50;
  pnc_map_->UpdateNextRoutingWaypointIndex(20);
  EXPECT_EQ(3, pnc_map_->next_routing_waypoint_index_);

  pnc_map_->adc_waypoint_.s = 100;
  pnc_map_->UpdateNextRoutingWaypointIndex(14);
  EXPECT_EQ(3, pnc_map_->next_routing_waypoint_index_);

  pnc_map_->adc_waypoint_.s = 60;
  pnc_map_->UpdateNextRoutingWaypointIndex(14);
  EXPECT_EQ(2, pnc_map_->next_routing_waypoint_index_);
}

TEST_F(PncMapTest, GetRouteSegments_ChangeLane) {
  auto lane = hdmap_.GetLaneById(hdmap::MakeMapId("9_1_-2"));
  ASSERT_TRUE(lane);
  common::VehicleState state;
  auto point = lane->GetSmoothPoint(35);  // larger than kMinLaneKeepingDistance
  state.set_x(point.x());
  state.set_y(point.y());
  state.set_z(point.y());
  state.set_heading(M_PI);
  std::list<RouteSegments> segments;
  bool result = pnc_map_->GetRouteSegments(state, 10, 30, &segments);
  ASSERT_TRUE(result);
  ASSERT_EQ(2, segments.size());
  const auto& first = segments.front();
  const auto& second = segments.back();
  EXPECT_NEAR(40, RouteLength(first), 1e-4);
  EXPECT_EQ(routing::LEFT, first.NextAction());
  EXPECT_TRUE(first.IsOnSegment());
  EXPECT_NEAR(40, RouteLength(second), 1e-4);
  EXPECT_EQ(routing::RIGHT, second.NextAction());
  EXPECT_FALSE(second.IsOnSegment());
}

TEST_F(PncMapTest, NextWaypointIndex) {
  EXPECT_EQ(0, pnc_map_->NextWaypointIndex(-2));
  EXPECT_EQ(0, pnc_map_->NextWaypointIndex(-1));
  EXPECT_EQ(1, pnc_map_->NextWaypointIndex(0));
  EXPECT_EQ(2, pnc_map_->NextWaypointIndex(1));
  EXPECT_EQ(3, pnc_map_->NextWaypointIndex(2));
  EXPECT_EQ(17, pnc_map_->NextWaypointIndex(18));
  EXPECT_EQ(17, pnc_map_->NextWaypointIndex(20));
}

TEST_F(PncMapTest, SearchForwardIndex_SearchBackwardIndex) {
  auto lane = hdmap_.GetLaneById(hdmap::MakeMapId("9_1_-2"));
  LaneWaypoint waypoint(lane, 3.0);
  auto result = pnc_map_->SearchForwardWaypointIndex(0, waypoint);
  EXPECT_EQ(11, result);
  result = pnc_map_->SearchBackwardWaypointIndex(0, waypoint);
  EXPECT_EQ(-1, result);
  result = pnc_map_->SearchForwardWaypointIndex(11, waypoint);
  EXPECT_EQ(11, result);
  result = pnc_map_->SearchBackwardWaypointIndex(11, waypoint);
  EXPECT_EQ(11, result);
  result = pnc_map_->SearchForwardWaypointIndex(12, waypoint);
  EXPECT_EQ(18, result);
  result = pnc_map_->SearchBackwardWaypointIndex(12, waypoint);
  EXPECT_EQ(11, result);
  result = pnc_map_->SearchForwardWaypointIndex(10, waypoint);
  EXPECT_EQ(11, result);
  result = pnc_map_->SearchBackwardWaypointIndex(10, waypoint);
  EXPECT_EQ(-1, result);
}

TEST_F(PncMapTest, GetNeighborPassages) {
  const auto& road0 = routing_.road(0);
  {
    auto result = pnc_map_->GetNeighborPassages(road0, 0);
    EXPECT_EQ(2, result.size());
    EXPECT_EQ(0, result[0]);
    EXPECT_EQ(1, result[1]);
  }
  {
    auto result = pnc_map_->GetNeighborPassages(road0, 1);
    EXPECT_EQ(3, result.size());
    EXPECT_EQ(1, result[0]);
    EXPECT_EQ(0, result[1]);
    EXPECT_EQ(2, result[2]);
  }
  {
    auto result = pnc_map_->GetNeighborPassages(road0, 2);
    EXPECT_EQ(1, result.size());
  }
  {
    auto result = pnc_map_->GetNeighborPassages(road0, 3);
    EXPECT_EQ(1, result.size());
    EXPECT_EQ(3, result[0]);
  }
}

}  // namespace hdmap
}  // namespace apollo
