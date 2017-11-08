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

#include "modules/common/util/file.h"
#include "modules/map/hdmap/hdmap_util.h"

namespace apollo {
namespace hdmap {
DEFINE_string(test_map_file, "modules/map/data/sunnyvale_loop/base_map.xml",
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
      CHECK(false);
    }
  }

  static hdmap::HDMap hdmap_;
};

hdmap::HDMap RouteSegmentsTest::hdmap_;

TEST_F(RouteSegmentsTest, RouteSegments_GetProjection) {
  auto lane1 = hdmap_.GetLaneById(hdmap::MakeMapId("9_1_-1"));
  RouteSegments route_segments;
  route_segments.emplace_back(lane1, 10, 20);
  LaneWaypoint waypoint;
  auto point = lane1->GetSmoothPoint(5);
  double s = 0.0;
  double l = 0.0;
  EXPECT_FALSE(route_segments.GetProjection(point, &s, &l, &waypoint));
  point = lane1->GetSmoothPoint(10);
  EXPECT_TRUE(route_segments.GetProjection(point, &s, &l, &waypoint));
  EXPECT_EQ(lane1, waypoint.lane);
  EXPECT_NEAR(10.0, waypoint.s, 1e-4);
  EXPECT_NEAR(0.0, s, 1e-4);
  EXPECT_NEAR(0.0, l, 1e-4);
  point = lane1->GetSmoothPoint(15);
  EXPECT_TRUE(route_segments.GetProjection(point, &s, &l, &waypoint));
  EXPECT_EQ(lane1, waypoint.lane);
  EXPECT_NEAR(15.0, waypoint.s, 1e-4);
  EXPECT_NEAR(5.0, s, 1e-4);
  EXPECT_NEAR(0.0, l, 1e-4);
  point = lane1->GetSmoothPoint(25);
  EXPECT_FALSE(route_segments.GetProjection(point, &s, &l, &waypoint));
  auto lane2 = hdmap_.GetLaneById(hdmap::MakeMapId("13_1_-1"));
  route_segments.emplace_back(lane2, 20, 30);
  EXPECT_FALSE(route_segments.GetProjection(point, &s, &l, &waypoint));
  point = lane2->GetSmoothPoint(0);
  EXPECT_FALSE(route_segments.GetProjection(point, &s, &l, &waypoint));
  point = lane2->GetSmoothPoint(25);
  EXPECT_TRUE(route_segments.GetProjection(point, &s, &l, &waypoint));
  EXPECT_EQ(lane2, waypoint.lane);
  EXPECT_NEAR(25.0, waypoint.s, 1e-4);
  EXPECT_NEAR(15.0, s, 1e-4);
  EXPECT_NEAR(0.0, l, 1e-4);
  point = lane2->GetSmoothPoint(31);
  EXPECT_FALSE(route_segments.GetProjection(point, &s, &l, &waypoint));
}

}  // namespace hdmap
}  // namespace apollo
