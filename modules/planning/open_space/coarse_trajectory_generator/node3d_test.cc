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

/*
 * @file
 */
#include "modules/planning/open_space/coarse_trajectory_generator/node3d.h"

#include "gtest/gtest.h"
#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/box2d.h"
#include "modules/common/math/vec2d.h"

namespace apollo {
namespace planning {

using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;

class Node3dTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    node_ = std::unique_ptr<Node3d>(new Node3d(0.0, 0.0, 0.0));
  }

 protected:
  std::unique_ptr<Node3d> node_;
  common::VehicleParam vehicle_param_ =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
};

TEST_F(Node3dTest, GetBoundingBox) {
  Box2d test_box = Node3d::GetBoundingBox(vehicle_param_, 0.0, 0.0, 0.0);
  double ego_length = vehicle_param_.length();
  double ego_width = vehicle_param_.width();
  Box2d gold_box({0.0, 0.0}, 0.0, ego_length, ego_width);
  double shift_distance =
      ego_length / 2.0 - vehicle_param_.back_edge_to_center();
  Vec2d shift_vec{shift_distance * std::cos(0.0),
                  shift_distance * std::sin(0.0)};
  gold_box.Shift(shift_vec);
  ASSERT_EQ(test_box.heading(), gold_box.heading());
  ASSERT_EQ(test_box.center_x(), gold_box.center_x());
  ASSERT_EQ(test_box.center_y(), gold_box.center_y());
  ASSERT_EQ(test_box.length(), gold_box.length());
  ASSERT_EQ(test_box.width(), gold_box.width());
}

}  // namespace planning
}  // namespace apollo
