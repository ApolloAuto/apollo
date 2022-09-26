/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/planner/public_road/public_road_planner.h"

#include "gtest/gtest.h"
#include "modules/common_msgs/basic_msgs/drive_state.pb.h"
#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

TEST(PublicRoadPlannerTest, Simple) {
  auto injector = std::make_shared<DependencyInjector>();
  PublicRoadPlanner public_road_planner(injector);
  PlanningConfig config;
  EXPECT_EQ(public_road_planner.Name(), "PUBLIC_ROAD");
  EXPECT_EQ(public_road_planner.Init(config), common::Status::OK());
}

}  // namespace planning
}  // namespace apollo
