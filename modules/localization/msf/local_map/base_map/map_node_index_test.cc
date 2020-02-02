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

#include "gtest/gtest.h"

#include "modules/localization/msf/local_map/base_map/base_map_node_index.h"

namespace apollo {
namespace localization {
namespace msf {

/**@brief Test the get method. */
TEST(BaseMapNodeIndexTestSuite, GetMethodTest) {
  MapNodeIndex node_index;
  EXPECT_EQ(node_index.resolution_id_, 0);
  EXPECT_EQ(node_index.zone_id_, 50);
  EXPECT_EQ(node_index.m_, 0);
  EXPECT_EQ(node_index.n_, 0);

  BaseMapConfig option;
  option.map_resolutions_.push_back(0.125);
  Eigen::Vector3d coordinate3d;
  coordinate3d << 435816.184008, 4435662.333578, 36.606647;
  MapNodeIndex node_index1 =
      MapNodeIndex::GetMapNodeIndex(option, coordinate3d, 0, 50);
  Eigen::Vector2d coordinate2d;
  coordinate2d << 435816.184008, 4435662.333578;
  MapNodeIndex node_index2 =
      MapNodeIndex::GetMapNodeIndex(option, coordinate2d, 0, 50);

  EXPECT_TRUE(node_index1 == node_index2);
}

/**@brief Test the operators. */
TEST(BaseMapNodeIndexTestSuite, OperatorTest) {
  MapNodeIndex node_index1;
  EXPECT_EQ(node_index1.resolution_id_, 0);
  EXPECT_EQ(node_index1.zone_id_, 50);
  EXPECT_EQ(node_index1.m_, 0);
  EXPECT_EQ(node_index1.n_, 0);

  MapNodeIndex node_index2;
  node_index2.n_ = 1;

  bool res = node_index1 < node_index2;
  EXPECT_TRUE(res);

  res = node_index1 != node_index2;
  EXPECT_TRUE(res);

  node_index2.n_ = 0;
  res = node_index1 == node_index2;
  EXPECT_TRUE(res);
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
