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

#include "modules/dreamview/backend/map/map_service.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "modules/common/configs/config_gflags.h"

using apollo::common::PointENU;
using apollo::hdmap::Map;

namespace apollo {
namespace dreamview {

class MapServiceTest : public ::testing::Test {
 protected:
  MapServiceTest() {
    FLAGS_map_dir = "modules/dreamview/backend/testdata";
    FLAGS_base_map_filename = "garage.bin";
    map_service.reset(new MapService(false));
  }

  std::unique_ptr<MapService> map_service;
};

TEST_F(MapServiceTest, CollectMapElementIds) {
  PointENU p;
  p.set_x(0.0);
  p.set_y(0.0);
  MapElementIds map_element_ids;
  map_service->CollectMapElementIds(p, 20000.0, &map_element_ids);

  EXPECT_EQ(1, map_element_ids.lane_size());
  EXPECT_EQ("l1", map_element_ids.lane(0));
  EXPECT_TRUE(map_element_ids.crosswalk().empty());
  EXPECT_TRUE(map_element_ids.junction().empty());
  EXPECT_TRUE(map_element_ids.signal().empty());
  EXPECT_TRUE(map_element_ids.stop_sign().empty());
  EXPECT_TRUE(map_element_ids.yield().empty());
}

TEST_F(MapServiceTest, RetrieveMapElements) {
  MapElementIds map_element_ids;
  map_element_ids.add_lane("l1");
  Map map = map_service->RetrieveMapElements(map_element_ids);
  EXPECT_EQ(1, map.lane_size());
  EXPECT_EQ("l1", map.lane(0).id().id());
}

TEST_F(MapServiceTest, GetStartPoint) {
  PointENU start_point;
  EXPECT_TRUE(map_service->GetStartPoint(&start_point));
  EXPECT_DOUBLE_EQ(-1826.4050789145094, start_point.x());
  EXPECT_DOUBLE_EQ(-3027.5187874953263, start_point.y());
  EXPECT_DOUBLE_EQ(0.0, start_point.z());
}

TEST_F(MapServiceTest, GetPoseWithRegardToLane) {
  double theta, s;
  EXPECT_TRUE(map_service->GetPoseWithRegardToLane(-1826, -3027, &theta, &s));
  EXPECT_DOUBLE_EQ(2.8771504789266014, theta);
  EXPECT_DOUBLE_EQ(0.0, s);
}

TEST_F(MapServiceTest, ConstructLaneWayPoint) {
  apollo::routing::LaneWaypoint way_point;
  EXPECT_TRUE(map_service->ConstructLaneWayPoint(-1826, -3027, &way_point));
  EXPECT_EQ("l1", way_point.id());
  EXPECT_DOUBLE_EQ(0.0, way_point.s());
  EXPECT_DOUBLE_EQ(-1826, way_point.pose().x());
  EXPECT_DOUBLE_EQ(-3027, way_point.pose().y());
}

TEST_F(MapServiceTest, CalculateMapHash) {
  MapElementIds ids;
  ids.add_lane("l1");
  size_t hash_code = map_service->CalculateMapHash(ids);
  EXPECT_EQ(7655793271563537204, hash_code);
}

}  // namespace dreamview
}  // namespace apollo
