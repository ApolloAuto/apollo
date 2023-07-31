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
#include "gtest/gtest.h"

#include "modules/perception/common/base/impending_collision_edge.h"

namespace apollo {
namespace perception {
namespace base {

class ImpendingCollisionEdgeTest : public ::testing::Test {};

TEST_F(ImpendingCollisionEdgeTest, ImpendingCollisionEdge) {
  ImpendingCollisionEdge edge;
  edge.id = 2;
  edge.tracking_time = 1.0;
  edge.points.push_back(Eigen::Vector3d(1, 1, 1));

  EXPECT_EQ(edge.id, 2);
  EXPECT_DOUBLE_EQ(edge.tracking_time, 1.0);
  EXPECT_EQ(edge.points.size(), 1);
  EXPECT_DOUBLE_EQ(edge.points[0].x(), 1);
}

TEST_F(ImpendingCollisionEdgeTest, ImpendingCollisionEdges) {
  ImpendingCollisionEdges edges;

  edges.timestamp = 1.0;
  std::shared_ptr<ImpendingCollisionEdge> collision_edge_ptr(
      new ImpendingCollisionEdge());
  edges.impending_collision_edges.push_back(collision_edge_ptr);

  EXPECT_DOUBLE_EQ(edges.timestamp, 1.0);
  EXPECT_EQ(edges.impending_collision_edges.size(), 1);
}

}  // namespace base
}  // namespace perception
}  // namespace apollo
