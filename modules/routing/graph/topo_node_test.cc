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

#include <string>

#include "modules/routing/graph/test_utils.h"

namespace apollo {
namespace routing {

TEST(TopoEdgeTestSuit, basic_test) {
  Node node_1;
  Node node_2;
  GetNodeForTest(&node_1, TEST_L1, TEST_R1);
  GetNodeForTest(&node_2, TEST_L2, TEST_R2);

  TopoNode topo_node_1(node_1);
  TopoNode topo_node_2(node_2);
  Edge edge;
  GetEdgeForTest(&edge, TEST_L1, TEST_L2, Edge::FORWARD);

  const TopoNode* tn_1 = &topo_node_1;
  const TopoNode* tn_2 = &topo_node_2;
  TopoEdge topo_edge(edge, tn_1, tn_2);

  ASSERT_EQ(edge.DebugString(), topo_edge.edge().DebugString());
  ASSERT_DOUBLE_EQ(TEST_EDGE_COST, topo_edge.cost());
  ASSERT_EQ(TEST_L1, topo_edge.from_lane_id());
  ASSERT_EQ(TEST_L2, topo_edge.to_lane_id());
  ASSERT_EQ(TET_FORWARD, topo_edge.type());

  ASSERT_EQ(tn_1, topo_edge.from_node());
  ASSERT_EQ(tn_2, topo_edge.to_node());
}

}  // namespace routing
}  // namespace apollo
