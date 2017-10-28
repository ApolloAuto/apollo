/* Copyright 2017 The Apollo Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
=========================================================================*/

#include <algorithm>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "modules/map/hdmap/hdmap_impl.h"

namespace {

constexpr char kMapFilename[] = "modules/map/hdmap/test-data/base_map.bin";

}  // namespace

namespace apollo {
namespace hdmap {

class HDMapImplTestSuite : public ::testing::Test {
 public:
  HDMapImplTestSuite() {
    EXPECT_EQ(0, hdmap_impl_.LoadMapFromFile(kMapFilename));
  }

 public:
  HDMapImpl hdmap_impl_;
};

TEST_F(HDMapImplTestSuite, GetLaneById) {
  Id lane_id;
  lane_id.set_id("1");
  EXPECT_TRUE(nullptr == hdmap_impl_.GetLaneById(lane_id));
  lane_id.set_id("1272_1_-1");
  LaneInfoConstPtr lane_ptr = hdmap_impl_.GetLaneById(lane_id);
  EXPECT_TRUE(nullptr != lane_ptr);
  EXPECT_STREQ(lane_id.id().c_str(), lane_ptr->id().id().c_str());
}

TEST_F(HDMapImplTestSuite, GetJunctionById) {
  Id junction_id;
  junction_id.set_id("1");
  EXPECT_TRUE(nullptr == hdmap_impl_.GetJunctionById(junction_id));
  junction_id.set_id("1183");
  JunctionInfoConstPtr junction_ptr = hdmap_impl_.GetJunctionById(junction_id);
  EXPECT_TRUE(nullptr != junction_ptr);
  EXPECT_STREQ(junction_id.id().c_str(), junction_ptr->id().id().c_str());
}

TEST_F(HDMapImplTestSuite, GetSignalById) {
  Id signal_id;
  signal_id.set_id("abc");
  EXPECT_TRUE(nullptr == hdmap_impl_.GetSignalById(signal_id));
  signal_id.set_id("1278");
  SignalInfoConstPtr signal_ptr = hdmap_impl_.GetSignalById(signal_id);
  EXPECT_TRUE(nullptr != signal_ptr);
  EXPECT_STREQ(signal_id.id().c_str(), signal_ptr->id().id().c_str());
}

TEST_F(HDMapImplTestSuite, GetCrosswalkById) {
  Id crosswalk_id;
  crosswalk_id.set_id("1");
  EXPECT_TRUE(nullptr == hdmap_impl_.GetCrosswalkById(crosswalk_id));
  crosswalk_id.set_id("1277");
  CrosswalkInfoConstPtr crosswalk_ptr =
      hdmap_impl_.GetCrosswalkById(crosswalk_id);
  EXPECT_TRUE(nullptr != crosswalk_ptr);
  EXPECT_STREQ(crosswalk_id.id().c_str(), crosswalk_ptr->id().id().c_str());
}

TEST_F(HDMapImplTestSuite, GetStopSignById) {
  Id stop_sign_id;
  stop_sign_id.set_id("1");
  EXPECT_TRUE(nullptr == hdmap_impl_.GetStopSignById(stop_sign_id));
  stop_sign_id.set_id("1276");
  StopSignInfoConstPtr stop_sign_ptr =
      hdmap_impl_.GetStopSignById(stop_sign_id);
  EXPECT_TRUE(nullptr != stop_sign_ptr);
  EXPECT_STREQ(stop_sign_id.id().c_str(), stop_sign_ptr->id().id().c_str());
}

TEST_F(HDMapImplTestSuite, GetYieldSignById) {
  Id yield_sign_id;
  yield_sign_id.set_id("1");
  EXPECT_TRUE(nullptr == hdmap_impl_.GetYieldSignById(yield_sign_id));
  yield_sign_id.set_id("1275");
  YieldSignInfoConstPtr yield_sign_ptr =
      hdmap_impl_.GetYieldSignById(yield_sign_id);
  EXPECT_TRUE(nullptr != yield_sign_ptr);
  EXPECT_STREQ(yield_sign_id.id().c_str(), yield_sign_ptr->id().id().c_str());
}

TEST_F(HDMapImplTestSuite, GetClearAreaById) {
  Id clear_area_id;
  clear_area_id.set_id("1");
  EXPECT_TRUE(nullptr == hdmap_impl_.GetClearAreaById(clear_area_id));
  clear_area_id.set_id("1254");
  ClearAreaInfoConstPtr clear_area_ptr =
      hdmap_impl_.GetClearAreaById(clear_area_id);
  EXPECT_TRUE(nullptr != clear_area_ptr);
  EXPECT_STREQ(clear_area_id.id().c_str(), clear_area_ptr->id().id().c_str());
}

TEST_F(HDMapImplTestSuite, GetSpeedBumpById) {
  Id speed_bump_id;
  speed_bump_id.set_id("1");
  EXPECT_TRUE(nullptr == hdmap_impl_.GetSpeedBumpById(speed_bump_id));
}

TEST_F(HDMapImplTestSuite, GetOverlapById) {
  Id overlap_id;
  overlap_id.set_id("1");
  EXPECT_TRUE(nullptr == hdmap_impl_.GetOverlapById(overlap_id));
  overlap_id.set_id("overlap_20");
  OverlapInfoConstPtr overlap_ptr = hdmap_impl_.GetOverlapById(overlap_id);
  EXPECT_TRUE(nullptr != overlap_ptr);
  EXPECT_STREQ(overlap_id.id().c_str(), overlap_ptr->id().id().c_str());
}

TEST_F(HDMapImplTestSuite, GetRoadById) {
  Id road_id;
  road_id.set_id("1");
  EXPECT_TRUE(nullptr == hdmap_impl_.GetRoadById(road_id));
  road_id.set_id("1272");
  RoadInfoConstPtr road_ptr = hdmap_impl_.GetRoadById(road_id);
  EXPECT_TRUE(nullptr != road_ptr);
  EXPECT_STREQ(road_id.id().c_str(), road_ptr->id().id().c_str());
}

TEST_F(HDMapImplTestSuite, GetLanes) {
  std::vector<LaneInfoConstPtr> lanes;
  apollo::common::PointENU point;
  point.set_x(586424.09);
  point.set_y(4140727.02);
  point.set_z(0.0);
  EXPECT_EQ(0, hdmap_impl_.GetLanes(point, 1e-6, &lanes));
  EXPECT_EQ(0, lanes.size());
  EXPECT_EQ(0, hdmap_impl_.GetLanes(point, 5, &lanes));
  EXPECT_EQ(1, lanes.size());
  std::vector<std::string> ids;
  for (const auto& lane : lanes) {
    ids.push_back(lane->id().id());
  }

  EXPECT_EQ("773_1_-2", ids[0]);
}

TEST_F(HDMapImplTestSuite, GetNearestLaneWithHeading) {
  apollo::common::PointENU point;
  point.set_x(586424.09);
  point.set_y(4140727.02);
  point.set_z(0.0);

  LaneInfoConstPtr nearest_lane;
  double nearest_s = 0.0;
  double nearest_l = 0.0;
  EXPECT_EQ(-1,
            hdmap_impl_.GetNearestLaneWithHeading(
                point, 1e-6, 0.86, 0.2, &nearest_lane, &nearest_s, &nearest_l));

  EXPECT_EQ(0,
            hdmap_impl_.GetNearestLaneWithHeading(
                point, 5, -2.35, 1.0, &nearest_lane, &nearest_s, &nearest_l));
  EXPECT_EQ("773_1_-2", nearest_lane->id().id());
  EXPECT_NEAR(nearest_l, -3.257, 1E-3);
  EXPECT_NEAR(nearest_s, 25.891, 1E-3);
}

TEST_F(HDMapImplTestSuite, GetLanesWithHeading) {
  apollo::common::PointENU point;
  point.set_x(586424.09);
  point.set_y(4140727.02);
  point.set_z(0.0);

  std::vector<LaneInfoConstPtr> lanes;
  EXPECT_EQ(-1,
            hdmap_impl_.GetLanesWithHeading(point, 1e-6, 0.86, 0.2, &lanes));

  EXPECT_EQ(0, hdmap_impl_.GetLanesWithHeading(point, 5, -2.35, 1.0, &lanes));
  EXPECT_EQ(1, lanes.size());
  EXPECT_EQ("773_1_-2", lanes[0]->id().id());
}

TEST_F(HDMapImplTestSuite, GetJunctions) {
  std::vector<JunctionInfoConstPtr> junctions;
  apollo::common::PointENU point;
  point.set_x(586441.61);
  point.set_y(4140746.48);
  point.set_z(0.0);
  EXPECT_EQ(0, hdmap_impl_.GetJunctions(point, 1, &junctions));
  EXPECT_EQ(0, junctions.size());
  EXPECT_EQ(0, hdmap_impl_.GetJunctions(point, 3, &junctions));
  EXPECT_EQ(1, junctions.size());
  EXPECT_EQ("1183", junctions[0]->id().id());
}

TEST_F(HDMapImplTestSuite, GetCrosswalks) {
  std::vector<CrosswalkInfoConstPtr> crosswalks;
  apollo::common::PointENU point;
  point.set_x(586449.32);
  point.set_y(4140789.59);
  point.set_z(0.0);
  EXPECT_EQ(0, hdmap_impl_.GetCrosswalks(point, 1, &crosswalks));
  EXPECT_EQ(0, crosswalks.size());
  EXPECT_EQ(0, hdmap_impl_.GetCrosswalks(point, 3, &crosswalks));
  EXPECT_EQ(1, crosswalks.size());
  EXPECT_EQ("1277", crosswalks[0]->id().id());
}

TEST_F(HDMapImplTestSuite, GetSignals) {
  std::vector<SignalInfoConstPtr> signals;

  apollo::common::PointENU point;
  point.set_x(586440.37);
  point.set_y(4140738.64);
  point.set_z(0.0);
  EXPECT_EQ(0, hdmap_impl_.GetSignals(point, 7.0, &signals));
  EXPECT_EQ(0, signals.size());
  EXPECT_EQ(0, hdmap_impl_.GetSignals(point, 12, &signals));
  EXPECT_EQ(1, signals.size());
  EXPECT_EQ("1278", signals[0]->id().id());
}

TEST_F(HDMapImplTestSuite, GetStopSigns) {
  std::vector<StopSignInfoConstPtr> stop_signs;

  apollo::common::PointENU point;
  point.set_x(586418.82);
  point.set_y(4140779.06);
  point.set_z(0.0);
  EXPECT_EQ(0, hdmap_impl_.GetStopSigns(point, 100.0, &stop_signs));
  EXPECT_EQ(1, stop_signs.size());
  EXPECT_EQ("1276", stop_signs[0]->id().id());
}

TEST_F(HDMapImplTestSuite, GetYieldSigns) {
  std::vector<YieldSignInfoConstPtr> yield_signs;

  apollo::common::PointENU point;
  point.set_x(586376.08);
  point.set_y(4140785.77);
  point.set_z(0.0);
  EXPECT_EQ(0, hdmap_impl_.GetYieldSigns(point, 100.0, &yield_signs));
  EXPECT_EQ(1, yield_signs.size());
  EXPECT_EQ("1275", yield_signs[0]->id().id());
}

TEST_F(HDMapImplTestSuite, GetClearAreas) {
  std::vector<ClearAreaInfoConstPtr> clear_areas;

  apollo::common::PointENU point;
  point.set_x(586426.24);
  point.set_y(4140680.01);
  point.set_z(0.0);
  EXPECT_EQ(0, hdmap_impl_.GetClearAreas(point, 4.0, &clear_areas));
  EXPECT_EQ(1, clear_areas.size());
  EXPECT_EQ("1254", clear_areas[0]->id().id());
}

TEST_F(HDMapImplTestSuite, GetSpeedBumps) {
  std::vector<SpeedBumpInfoConstPtr> speed_bumps;

  apollo::common::PointENU point;
  point.set_x(586410.13);
  point.set_y(4140679.01);
  point.set_z(0.0);
  EXPECT_EQ(0, hdmap_impl_.GetSpeedBumps(point, 4.0, &speed_bumps));
  EXPECT_EQ(0, speed_bumps.size());
}

TEST_F(HDMapImplTestSuite, GetRoads) {
  std::vector<RoadInfoConstPtr> roads;

  apollo::common::PointENU point;
  point.set_x(586427.18);
  point.set_y(4140741.36);
  point.set_z(0.0);
  EXPECT_EQ(0, hdmap_impl_.GetRoads(point, 4.0, &roads));
  EXPECT_EQ(1, roads.size());

  EXPECT_EQ(roads[0]->id().id(), "773");
  EXPECT_TRUE(!roads[0]->has_junction_id());

  EXPECT_EQ(1, roads[0]->sections().size());
  const apollo::hdmap::RoadSection& section = roads[0]->sections()[0];
  EXPECT_EQ(section.id().id(), "1");
  EXPECT_EQ(section.lane_id_size(), 2);

  std::set<std::string> lane_ids;
  lane_ids.insert("773_1_-1");
  lane_ids.insert("773_1_-2");
  for (int i = 0; i < section.lane_id_size(); ++i) {
    const apollo::hdmap::Id& lane_id = section.lane_id(i);
    EXPECT_GT(lane_ids.count(lane_id.id()), 0);
  }
}

TEST_F(HDMapImplTestSuite, GetNearestLane) {
  LaneInfoConstPtr lane;
  double s = 0.0;
  double l = 0.0;

  apollo::common::PointENU point;
  point.set_x(586424.09);
  point.set_y(4140727.02);
  point.set_z(0.0);
  EXPECT_EQ(0, hdmap_impl_.GetNearestLane(point, &lane, &s, &l));
  EXPECT_EQ("773_1_-2", lane->id().id());
  EXPECT_NEAR(s, 25.891, 1e-3);
  EXPECT_NEAR(l, -3.257, 1e-3);
}

TEST_F(HDMapImplTestSuite, GetRoadBoundaries) {
  apollo::common::PointENU point;
  point.set_x(586427.58);
  point.set_y(4140749.35);
  point.set_z(0.0);
  std::vector<RoadROIBoundaryPtr> road_boundaries;
  std::vector<JunctionBoundaryPtr> junctions;
  EXPECT_EQ(-1, hdmap_impl_.GetRoadBoundaries(point, 3.0, &road_boundaries,
                                              &junctions));

  point.set_x(586434.75);
  point.set_y(4140746.94);
  point.set_z(0.0);
  EXPECT_EQ(0, hdmap_impl_.GetRoadBoundaries(point, 2.0, &road_boundaries,
                                             &junctions));
  EXPECT_EQ(1, road_boundaries.size());
  EXPECT_EQ(0, junctions.size());

  point.set_x(586434.75);
  point.set_y(4140746.94);
  point.set_z(0.0);
  EXPECT_EQ(0, hdmap_impl_.GetRoadBoundaries(point, 4.5, &road_boundaries,
                                             &junctions));
  EXPECT_EQ(1, road_boundaries.size());
  EXPECT_EQ(1, junctions.size());
}

}  // namespace hdmap
}  // namespace apollo
