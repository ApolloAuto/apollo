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

#include "modules/routing/common/routing_gflags.h"
#include "modules/routing/graph/topo_test_utils.h"

namespace apollo {
namespace routing {

namespace {

void GetRangeVec(std::vector<NodeSRange>* range_vec) {
  double s_s_1 = 1.0;
  double e_s_1 = 12.0;
  double s_s_2 = 15.0;
  double e_s_2 = 27.0;
  double s_s_3 = 30.0;
  double e_s_3 = 45.1;
  double s_s_4 = 55.5;
  double e_s_4 = 70.0;

  range_vec->push_back(NodeSRange(s_s_2, e_s_2));
  range_vec->push_back(NodeSRange(s_s_4, e_s_4));
  range_vec->push_back(NodeSRange(s_s_1, e_s_1));
  range_vec->push_back(NodeSRange(s_s_3, e_s_3));

  sort(range_vec->begin(), range_vec->end());

  ASSERT_DOUBLE_EQ(s_s_1, range_vec->at(0).StartS());
  ASSERT_DOUBLE_EQ(e_s_1, range_vec->at(0).EndS());
  ASSERT_DOUBLE_EQ(s_s_2, range_vec->at(1).StartS());
  ASSERT_DOUBLE_EQ(e_s_2, range_vec->at(1).EndS());
  ASSERT_DOUBLE_EQ(s_s_3, range_vec->at(2).StartS());
  ASSERT_DOUBLE_EQ(e_s_3, range_vec->at(2).EndS());
  ASSERT_DOUBLE_EQ(s_s_4, range_vec->at(3).StartS());
  ASSERT_DOUBLE_EQ(e_s_4, range_vec->at(3).EndS());
}

}  // namespace

TEST(TopoNodeTestSuit, static_func_test) {
  FLAGS_min_length_for_lane_change = 10.0;
  std::vector<NodeSRange> range_vec;
  GetRangeVec(&range_vec);
  {
    double start_s = 13.0;
    double end_s = 26.0;
    ASSERT_TRUE(TopoNode::IsOutRangeEnough(range_vec, start_s, end_s));
  }
  {
    double start_s = 13.0;
    double end_s = 23.0;
    ASSERT_FALSE(TopoNode::IsOutRangeEnough(range_vec, start_s, end_s));
  }
  {
    double start_s = 22.0;
    double end_s = 32.0;
    ASSERT_FALSE(TopoNode::IsOutRangeEnough(range_vec, start_s, end_s));
  }
  {
    double start_s = 31.0;
    double end_s = 44.0;
    ASSERT_TRUE(TopoNode::IsOutRangeEnough(range_vec, start_s, end_s));
  }
  {
    double start_s = -10;
    double end_s = 100;
    ASSERT_TRUE(TopoNode::IsOutRangeEnough(range_vec, start_s, end_s));
  }
}

TEST(TopoNodeTestSuit, basic_test) {
  Node node;
  GetNodeDetailForTest(&node, TEST_L1, TEST_R1);
  TopoNode topo_node(node);
  ASSERT_EQ(node.DebugString(), topo_node.PbNode().DebugString());
  ASSERT_EQ(TEST_L1, topo_node.LaneId());
  ASSERT_EQ(TEST_R1, topo_node.RoadId());
  ASSERT_DOUBLE_EQ(TEST_MIDDLE_S, topo_node.AnchorPoint().x());
  ASSERT_DOUBLE_EQ(0.0, topo_node.AnchorPoint().y());
  ASSERT_DOUBLE_EQ(TEST_LANE_LENGTH, topo_node.Length());
  ASSERT_DOUBLE_EQ(TEST_LANE_COST, topo_node.Cost());
  ASSERT_TRUE(topo_node.IsVirtual());
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
