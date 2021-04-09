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

#include "modules/map/hdmap/hdmap_util.h"

#include "gtest/gtest.h"
#include "modules/common/time/time.h"

#include "modules/common/adapters/adapter_manager.h"

namespace apollo {
namespace hdmap {
using apollo::common::adapter::AdapterConfig;
using apollo::common::adapter::AdapterManager;
using apollo::common::adapter::AdapterManagerConfig;
using apollo::common::time::Clock;
using apollo::relative_map::MapMsg;

class HDMapUtilTestSuite : public ::testing::Test {
 protected:
  HDMapUtilTestSuite() {}
  virtual ~HDMapUtilTestSuite() {}
  virtual void SetUp() {}
  virtual void TearDown() {}
  void InitMapProto(Map* map_proto);
};

void HDMapUtilTestSuite::InitMapProto(Map* map_proto) {
  auto* lane = map_proto->add_lane();
  lane->mutable_id()->set_id("lane_1");
  CurveSegment* curve_segment = lane->mutable_central_curve()->add_segment();
  LineSegment* line_segment = curve_segment->mutable_line_segment();
  double delta_s = 0.2;
  double heading = M_PI / 2.0;
  double x0 = 0.0;
  double y0 = 0.0;
  for (double s = 0; s < 200; s += delta_s) {
    auto* pt = line_segment->add_point();
    pt->set_x(x0 + s * cos(heading));
    pt->set_y(y0 + s * sin(heading));
    auto* left_sample = lane->add_left_sample();
    left_sample->set_s(s);
    left_sample->set_width(1.5);
    auto* right_sample = lane->add_right_sample();
    right_sample->set_s(s);
    right_sample->set_width(1.5);
  }
  lane->set_type(Lane::CITY_DRIVING);
}

TEST_F(HDMapUtilTestSuite, ReuseMap) {
  MapMsg map_msg;
  InitMapProto(map_msg.mutable_hdmap());
  map_msg.mutable_header()->set_sequence_num(1);
  FLAGS_use_navigation_mode = true;
  AdapterManagerConfig adapter_config;
  adapter_config.set_is_ros(false);
  auto* config = adapter_config.add_config();
  config->set_type(AdapterConfig::RELATIVE_MAP);
  config->set_mode(AdapterConfig::RECEIVE_ONLY);
  AdapterManager::Init(adapter_config);
  AdapterManager::FeedRelativeMapData(map_msg);
  AdapterManager::Observe();

  // First time generate map: 4ms dbg / 0.8ms opt
  auto* hdmap = HDMapUtil::BaseMapPtr();
  ASSERT_TRUE(hdmap != nullptr);
  // Second time runtime of reuse a ptr is 1/1000 ms.
  auto* hdmap1 = HDMapUtil::BaseMapPtr();
  EXPECT_EQ(hdmap, hdmap1);  // should be the same ptr.
  map_msg.mutable_header()->set_sequence_num(2);
  AdapterManager::FeedRelativeMapData(map_msg);
  AdapterManager::Observe();
  auto* hdmap2 = HDMapUtil::BaseMapPtr();
  EXPECT_NE(hdmap2, hdmap1);  // hdmap should be updated.
}

}  // namespace hdmap
}  // namespace apollo
