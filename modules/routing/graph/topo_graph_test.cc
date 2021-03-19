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

#include "modules/routing/graph/topo_graph.h"

#include "gtest/gtest.h"
#include "modules/routing/graph/topo_test_utils.h"

namespace apollo {
namespace routing {

TEST(TopoGraphTestSuit, test_graph_1) {
  Graph graph;
  GetGraphForTest(&graph);

  TopoGraph topo_graph;
  ASSERT_TRUE(topo_graph.LoadGraph(graph));

  ASSERT_EQ(TEST_MAP_VERSION, topo_graph.MapVersion());
  ASSERT_EQ(TEST_MAP_DISTRICT, topo_graph.MapDistrict());

  const TopoNode* node_1 = topo_graph.GetNode(TEST_L1);
  ASSERT_TRUE(node_1 != nullptr);
  ASSERT_EQ(1, node_1->InFromAllEdge().size());
  ASSERT_EQ(1, node_1->InFromRightEdge().size());
  ASSERT_EQ(2, node_1->OutToAllEdge().size());
  ASSERT_EQ(1, node_1->OutToSucEdge().size());
  ASSERT_EQ(1, node_1->OutToRightEdge().size());
  ASSERT_DOUBLE_EQ(TEST_LANE_COST, node_1->Cost());
  ASSERT_EQ(TEST_L1, node_1->LaneId());
  ASSERT_EQ(TEST_R1, node_1->RoadId());
  ASSERT_EQ(node_1, node_1->OriginNode());
  ASSERT_DOUBLE_EQ(0.0, node_1->StartS());
  ASSERT_DOUBLE_EQ(TEST_LANE_LENGTH, node_1->EndS());
  ASSERT_DOUBLE_EQ(TEST_LANE_LENGTH, node_1->Length());
  ASSERT_FALSE(node_1->IsSubNode());

  const TopoNode* node_4 = topo_graph.GetNode(TEST_L4);
  ASSERT_TRUE(node_4 != nullptr);
  ASSERT_EQ(2, node_4->InFromAllEdge().size());
  ASSERT_EQ(1, node_4->InFromPreEdge().size());
  ASSERT_EQ(1, node_4->InFromLeftEdge().size());
  ASSERT_EQ(1, node_4->OutToAllEdge().size());
  ASSERT_EQ(1, node_4->OutToLeftEdge().size());
  ASSERT_DOUBLE_EQ(TEST_LANE_COST, node_4->Cost());
  ASSERT_EQ(TEST_L4, node_4->LaneId());
  ASSERT_EQ(TEST_R2, node_4->RoadId());
  ASSERT_EQ(node_4, node_4->OriginNode());
  ASSERT_DOUBLE_EQ(0.0, node_4->StartS());
  ASSERT_DOUBLE_EQ(TEST_LANE_LENGTH, node_4->EndS());
  ASSERT_DOUBLE_EQ(TEST_LANE_LENGTH, node_4->Length());
  ASSERT_FALSE(node_4->IsSubNode());
}

}  // namespace routing
}  // namespace apollo
