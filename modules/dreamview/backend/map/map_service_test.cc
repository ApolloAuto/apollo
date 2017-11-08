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

using apollo::hdmap::Id;
using apollo::hdmap::Map;
using apollo::common::PointENU;
using ::testing::UnorderedElementsAre;

namespace apollo {
namespace dreamview {

TEST(MapElementIdsTest, Hash) {
  MapElementIds ids;
  ids.lane = {"first_lane", "second_lane", "haha_lane"};
  ids.overlap = {"last_overlap"};
  EXPECT_EQ(
      std::hash<std::string>()("first_lanesecond_lanehaha_lanelast_overlap"),
      ids.Hash());
}

TEST(MapElementIdsTest, Json) {
  MapElementIds ids;
  ids.lane = {"first_lane", "second_lane", "haha_lane"};
  auto json = ids.Json();

  EXPECT_EQ(
      "{\"crosswalk\":[],\"junction\":[],\"lane\":[\"first_lane\","
      "\"second_lane\",\"haha_lane\"],\"overlap\":[],\"signal\":[],"
      "\"stopSign\":[],\"yield\":[]}",
      json.dump());

  MapElementIds from_json(json);
  EXPECT_THAT(from_json.lane,
              UnorderedElementsAre("first_lane", "second_lane", "haha_lane"));
  EXPECT_THAT(from_json.crosswalk, UnorderedElementsAre());
  EXPECT_THAT(from_json.junction, UnorderedElementsAre());
  EXPECT_THAT(from_json.signal, UnorderedElementsAre());
  EXPECT_THAT(from_json.stop_sign, UnorderedElementsAre());
  EXPECT_THAT(from_json.yield, UnorderedElementsAre());
  EXPECT_THAT(from_json.overlap, UnorderedElementsAre());
}

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
  MapElementIds map_element_ids = map_service->CollectMapElementIds(p, 20000.0);

  EXPECT_THAT(map_element_ids.lane, UnorderedElementsAre("l1"));
  EXPECT_THAT(map_element_ids.crosswalk, UnorderedElementsAre());
  EXPECT_THAT(map_element_ids.junction, UnorderedElementsAre());
  EXPECT_THAT(map_element_ids.signal, UnorderedElementsAre());
  EXPECT_THAT(map_element_ids.stop_sign, UnorderedElementsAre());
  EXPECT_THAT(map_element_ids.yield, UnorderedElementsAre());
  EXPECT_THAT(map_element_ids.overlap, UnorderedElementsAre());
}

TEST_F(MapServiceTest, RetrieveMapElements) {
  MapElementIds map_element_ids;
  map_element_ids.lane.push_back("l1");
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

}  // namespace dreamview
}  // namespace apollo
