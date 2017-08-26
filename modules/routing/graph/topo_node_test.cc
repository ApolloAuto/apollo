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

#include "gtest/gtest.h"
#include "modules/routing/graph/topo_graph.h"
#include "modules/routing/graph/topo_node.h"

namespace apollo {
namespace routing {

const std::string TEST_L1 = "L1";
const std::string TEST_L2 = "L2";
const std::string TEST_R1 = "R1";
const std::string TEST_R2 = "R2";

const double TEST_LANE_LENGTH = 100.0;
const double TEST_LANE_COST = 1.1;
const double TEST_EDGE_COST = 2.2;

const double TEST_START_S = 0.0;
const double TEST_END_S = TEST_LANE_LENGTH;

void GetNodeForTest(Node* const node,
                    const std::string& lane_id, const std::string& road_id) {
  node->set_lane_id(lane_id);
  node->set_length(TEST_LANE_LENGTH);
  node->set_road_id(road_id);
  node->set_cost(TEST_LANE_COST);
  auto* left_out = node->add_left_out();
  left_out->mutable_start()->set_s(TEST_START_S);
  left_out->mutable_end()->set_s(TEST_END_S);
  auto* right_out = node->add_right_out();
  right_out->mutable_start()->set_s(TEST_START_S);
  right_out->mutable_end()->set_s(TEST_END_S);
}

void GetEdgeForTest(Edge* const edge,
                    const std::string& lane_id_1, const std::string& lane_id_2,
                    const Edge::DirectionType& type) {
  edge->set_from_lane_id(lane_id_1);
  edge->set_to_lane_id(lane_id_2);
  edge->set_cost(TEST_EDGE_COST);
  edge->set_direction_type(type);
}

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

  ASSERT_EQ(edge.DebugString(), topo_edge.PbEdge().DebugString());
  ASSERT_DOUBLE_EQ(TEST_EDGE_COST, topo_edge.Cost());
  ASSERT_EQ(TEST_L1, topo_edge.FromLaneId());
  ASSERT_EQ(TEST_L2, topo_edge.ToLaneId());
  ASSERT_EQ(TET_FORWARD, topo_edge.Type());

  ASSERT_EQ(tn_1, topo_edge.FromNode());
  ASSERT_EQ(tn_2, topo_edge.ToNode());
}

}  // namespace routing
}  // namespace apollo
