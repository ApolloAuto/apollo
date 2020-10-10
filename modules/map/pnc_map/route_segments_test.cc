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

#include "modules/map/pnc_map/route_segments.h"

#include "gflags/gflags.h"
#include "gtest/gtest.h"

#include "cyber/common/file.h"
#include "modules/map/hdmap/hdmap_util.h"

namespace apollo {
namespace hdmap {
DEFINE_string(test_map_file,
              "modules/map/data/sunnyvale_loop/base_map_test.bin",
              "The test map file");
DEFINE_string(
    test_routing_file,
    "modules/map/pnc_map/testdata/sample_sunnyvale_loop_routing.pb.txt",
    "The test map file");

class RouteSegmentsTest : public ::testing::Test {
 public:
  static void SetUpTestCase() {
    AINFO << "map file: " << FLAGS_test_map_file;
    if (hdmap_.LoadMapFromFile(FLAGS_test_map_file) != 0) {
      AERROR << "Failed to load map: " << FLAGS_test_map_file;
      ACHECK(false);
    }
  }

  static hdmap::HDMap hdmap_;
};

hdmap::HDMap RouteSegmentsTest::hdmap_;

TEST_F(RouteSegmentsTest, GetProjection) {
  auto lane1 = hdmap_.GetLaneById(hdmap::MakeMapId("9_1_-1"));
  RouteSegments route_segments;
  route_segments.emplace_back(lane1, 5, 10);
  LaneWaypoint waypoint;
  auto point = lane1->GetSmoothPoint(3);
  common::SLPoint sl;
  EXPECT_FALSE(route_segments.GetProjection(point, &sl, &waypoint));
  point = lane1->GetSmoothPoint(5);
  EXPECT_TRUE(route_segments.GetProjection(point, &sl, &waypoint));
  EXPECT_EQ(lane1, waypoint.lane);
  EXPECT_NEAR(5.0, waypoint.s, 1e-4);
  EXPECT_NEAR(0.0, sl.s(), 1e-4);
  EXPECT_NEAR(0.0, sl.l(), 1e-4);
  point = lane1->GetSmoothPoint(8);
  EXPECT_TRUE(route_segments.GetProjection(point, &sl, &waypoint));
  EXPECT_EQ(lane1, waypoint.lane);
  EXPECT_NEAR(8.0, waypoint.s, 1e-4);
  EXPECT_NEAR(3.0, sl.s(), 1e-4);
  EXPECT_NEAR(0.0, sl.l(), 1e-4);
  point = lane1->GetSmoothPoint(15);
  EXPECT_FALSE(route_segments.GetProjection(point, &sl, &waypoint));
  auto lane2 = hdmap_.GetLaneById(hdmap::MakeMapId("13_1_-1"));
  route_segments.emplace_back(lane2, 20, 30);
  EXPECT_FALSE(route_segments.GetProjection(point, &sl, &waypoint));
  point = lane2->GetSmoothPoint(0);
  EXPECT_FALSE(route_segments.GetProjection(point, &sl, &waypoint));
  point = lane2->GetSmoothPoint(25);
  EXPECT_TRUE(route_segments.GetProjection(point, &sl, &waypoint));
  EXPECT_EQ(lane2, waypoint.lane);
  EXPECT_NEAR(25.0, waypoint.s, 1e-4);
  EXPECT_NEAR(10.0, sl.s(), 1e-4);
  EXPECT_NEAR(0.0, sl.l(), 1e-4);
  point = lane2->GetSmoothPoint(31);
  EXPECT_FALSE(route_segments.GetProjection(point, &sl, &waypoint));
}

TEST_F(RouteSegmentsTest, Stitch) {
  auto lane1 = hdmap_.GetLaneById(hdmap::MakeMapId("9_1_-1"));
  auto lane2 = hdmap_.GetLaneById(hdmap::MakeMapId("13_1_-1"));
  {
    RouteSegments seg1;
    RouteSegments seg2;
    seg1.emplace_back(lane1, 10, 20);
    seg1.emplace_back(lane2, 10, 15);
    seg2.emplace_back(lane2, 15, 20);
    seg2.emplace_back(lane2, 20, 30);
    EXPECT_TRUE(seg1.Stitch(seg2));
    EXPECT_EQ(3, seg1.size());
    EXPECT_EQ(lane1, seg1[0].lane);
    EXPECT_FLOAT_EQ(10, seg1[0].start_s);
    EXPECT_FLOAT_EQ(20, seg1[0].end_s);
    EXPECT_EQ(lane2, seg1[1].lane);
    EXPECT_FLOAT_EQ(10, seg1[1].start_s);
    EXPECT_FLOAT_EQ(20, seg1[1].end_s);
    EXPECT_EQ(lane2, seg1[2].lane);
    EXPECT_FLOAT_EQ(20, seg1[2].start_s);
    EXPECT_FLOAT_EQ(30, seg1[2].end_s);
  }
  {
    RouteSegments seg1;
    RouteSegments seg2;
    seg1.emplace_back(lane1, 10, 20);
    seg1.emplace_back(lane2, 10, 15);
    seg2.emplace_back(lane2, 15, 20);
    seg2.emplace_back(lane2, 20, 30);
    EXPECT_TRUE(seg2.Stitch(seg1));
    EXPECT_EQ(3, seg2.size());
    EXPECT_EQ(lane1, seg2[0].lane);
    EXPECT_FLOAT_EQ(10, seg2[0].start_s);
    EXPECT_FLOAT_EQ(20, seg2[0].end_s);
    EXPECT_EQ(lane2, seg2[1].lane);
    EXPECT_FLOAT_EQ(10, seg2[1].start_s);
    EXPECT_FLOAT_EQ(20, seg2[1].end_s);
    EXPECT_EQ(lane2, seg2[2].lane);
    EXPECT_FLOAT_EQ(20, seg2[2].start_s);
    EXPECT_FLOAT_EQ(30, seg2[2].end_s);
  }
}

TEST_F(RouteSegmentsTest, Shrink) {
  auto lane1 = hdmap_.GetLaneById(hdmap::MakeMapId("9_1_-1"));
  auto lane2 = hdmap_.GetLaneById(hdmap::MakeMapId("13_1_-1"));
  {
    RouteSegments seg;
    seg.emplace_back(lane1, 10, 20);
    seg.emplace_back(lane2, 15, 20);
    seg.emplace_back(lane2, 20, 30);
    auto point = lane2->GetSmoothPoint(15.0);
    EXPECT_TRUE(seg.Shrink({point.x(), point.y()}, 5, 10));
    EXPECT_EQ(3, seg.size());
    EXPECT_EQ(lane1, seg[0].lane);
    EXPECT_FLOAT_EQ(15, seg[0].start_s);
    EXPECT_FLOAT_EQ(20, seg[0].end_s);
    EXPECT_EQ(lane2, seg[1].lane);
    EXPECT_FLOAT_EQ(15, seg[1].start_s);
    EXPECT_FLOAT_EQ(20, seg[1].end_s);
    EXPECT_FLOAT_EQ(20, seg[2].start_s);
    EXPECT_FLOAT_EQ(25, seg[2].end_s);
  }
  {
    RouteSegments seg;
    seg.emplace_back(lane1, 10, 20);
    seg.emplace_back(lane2, 15, 20);
    seg.emplace_back(lane2, 20, 30);
    auto point = lane2->GetSmoothPoint(15.0);
    EXPECT_TRUE(seg.Shrink({point.x(), point.y()}, 50, 10));
    EXPECT_EQ(3, seg.size());
    EXPECT_EQ(lane1, seg[0].lane);
    EXPECT_FLOAT_EQ(10, seg[0].start_s);
    EXPECT_FLOAT_EQ(20, seg[0].end_s);
    EXPECT_EQ(lane2, seg[1].lane);
    EXPECT_FLOAT_EQ(15, seg[1].start_s);
    EXPECT_FLOAT_EQ(20, seg[1].end_s);
    EXPECT_EQ(lane2, seg[2].lane);
    EXPECT_FLOAT_EQ(20, seg[2].start_s);
    EXPECT_FLOAT_EQ(25, seg[2].end_s);
  }
  {
    RouteSegments seg;
    seg.emplace_back(lane1, 10, 20);
    seg.emplace_back(lane2, 15, 20);
    seg.emplace_back(lane2, 20, 30);
    auto point = lane2->GetSmoothPoint(15.0);
    EXPECT_TRUE(seg.Shrink({point.x(), point.y()}, -5, 50));
    EXPECT_EQ(1, seg.size());
    EXPECT_EQ(lane2, seg[0].lane);
    EXPECT_FLOAT_EQ(20, seg[0].start_s);
    EXPECT_FLOAT_EQ(30, seg[0].end_s);
  }
  {
    RouteSegments seg;
    seg.emplace_back(lane2, 10, 30);
    auto point = lane2->GetSmoothPoint(20.0);
    EXPECT_TRUE(seg.Shrink({point.x(), point.y()}, 5, 5));
    EXPECT_EQ(1, seg.size());
    EXPECT_EQ(lane2, seg[0].lane);
    EXPECT_FLOAT_EQ(15, seg[0].start_s);
    EXPECT_FLOAT_EQ(25, seg[0].end_s);
  }
}

}  // namespace hdmap
}  // namespace apollo
