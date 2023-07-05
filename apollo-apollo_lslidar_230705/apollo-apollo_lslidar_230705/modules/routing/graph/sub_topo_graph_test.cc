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

#include "modules/routing/graph/sub_topo_graph.h"

#include "gtest/gtest.h"
#include "modules/routing/graph/topo_graph.h"
#include "modules/routing/graph/topo_test_utils.h"

namespace apollo {
namespace routing {

namespace {

NodeSRange GetSRange(double start_s, double end_s) {
  NodeSRange range(start_s, end_s);
  return range;
}

void GetTopoGraph(TopoGraph* topo_graph) {
  Graph graph;
  GetGraph2ForTest(&graph);
  ASSERT_TRUE(topo_graph->LoadGraph(graph));
  ASSERT_EQ(TEST_MAP_VERSION, topo_graph->MapVersion());
  ASSERT_EQ(TEST_MAP_DISTRICT, topo_graph->MapDistrict());
}

}  // namespace

TEST(SubTopoGraphTestSuit, one_sub_graph_all_valid) {
  TopoGraph topo_graph;
  GetTopoGraph(&topo_graph);

  const TopoNode* node_1 = topo_graph.GetNode(TEST_L1);
  ASSERT_TRUE(node_1 != nullptr);
  const TopoNode* node_2 = topo_graph.GetNode(TEST_L2);
  ASSERT_TRUE(node_2 != nullptr);

  std::unordered_map<const TopoNode*, std::vector<NodeSRange>> range_list;
  range_list[node_2].push_back(GetSRange(20.0, 50.0));
  SubTopoGraph sub_topo_graph(range_list);

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

  const TopoEdge* edge_1_2 = node_1->GetOutEdgeTo(node_2);
  ASSERT_TRUE(edge_1_2 != nullptr);

  std::unordered_set<const TopoEdge*> sub_edges_1_2;
  sub_topo_graph.GetSubInEdgesIntoSubGraph(edge_1_2, &sub_edges_1_2);
  ASSERT_EQ(2, sub_edges_1_2.size());

  double min_start_s = std::numeric_limits<double>::max();
  double max_start_s = 0.0;
  double min_end_s = std::numeric_limits<double>::max();
  double max_end_s = 0.0;
  for (const auto* edge : sub_edges_1_2) {
    ASSERT_EQ(node_1, edge->FromNode());
    const auto* to_node = edge->ToNode();
    min_start_s = std::min(to_node->StartS(), min_start_s);
    max_start_s = std::max(to_node->StartS(), max_start_s);
    min_end_s = std::min(to_node->EndS(), min_end_s);
    max_end_s = std::max(to_node->EndS(), max_end_s);
  }

  ASSERT_DOUBLE_EQ(0.0, min_start_s);
  ASSERT_DOUBLE_EQ(20.0, min_end_s);
  ASSERT_DOUBLE_EQ(50.0, max_start_s);
  ASSERT_DOUBLE_EQ(TEST_LANE_LENGTH, max_end_s);

  const TopoEdge* edge_2_1 = node_2->GetOutEdgeTo(node_1);
  ASSERT_TRUE(edge_2_1 != nullptr);

  std::unordered_set<const TopoEdge*> sub_edges_2_1;
  sub_topo_graph.GetSubInEdgesIntoSubGraph(edge_2_1, &sub_edges_2_1);
  // return origin edge when A has sub node, B has no sub node
  ASSERT_EQ(1, sub_edges_2_1.size());
  const auto* out_edge = *(sub_edges_2_1.begin());
  ASSERT_EQ(edge_2_1, out_edge);

  const TopoNode* node_4 = topo_graph.GetNode(TEST_L4);
  ASSERT_TRUE(node_4 != nullptr);
  const TopoEdge* edge_2_4 = node_2->GetOutEdgeTo(node_4);

  std::unordered_set<const TopoEdge*> sub_edges_2_4;
  sub_topo_graph.GetSubOutEdgesIntoSubGraph(edge_2_4, &sub_edges_2_4);
  ASSERT_EQ(1, sub_edges_2_4.size());
  const auto* sub_node_in_2 = (*(sub_edges_2_4.begin()))->FromNode();
  ASSERT_EQ(50.0, sub_node_in_2->StartS());
  ASSERT_EQ(TEST_LANE_LENGTH, sub_node_in_2->EndS());

  ASSERT_EQ(1, sub_node_in_2->InFromAllEdge().size());
  ASSERT_EQ(1, sub_node_in_2->InFromLeftEdge().size());
  ASSERT_EQ(2, sub_node_in_2->OutToAllEdge().size());
  ASSERT_EQ(1, sub_node_in_2->OutToLeftEdge().size());
  ASSERT_EQ(1, sub_node_in_2->OutToSucEdge().size());
  ASSERT_DOUBLE_EQ(TEST_LANE_COST, sub_node_in_2->Cost());
  ASSERT_EQ(TEST_L2, sub_node_in_2->LaneId());
  ASSERT_EQ(TEST_R1, sub_node_in_2->RoadId());
  ASSERT_EQ(node_2, sub_node_in_2->OriginNode());
  ASSERT_DOUBLE_EQ(TEST_LANE_LENGTH, sub_node_in_2->Length());
  ASSERT_TRUE(sub_node_in_2->IsSubNode());
}

TEST(SubTopoGraphTestSuit, one_sub_graph_pre_valid) {
  TopoGraph topo_graph;
  GetTopoGraph(&topo_graph);

  const TopoNode* node_1 = topo_graph.GetNode(TEST_L1);
  ASSERT_TRUE(node_1 != nullptr);
  const TopoNode* node_2 = topo_graph.GetNode(TEST_L2);
  ASSERT_TRUE(node_2 != nullptr);

  std::unordered_map<const TopoNode*, std::vector<NodeSRange>> range_list;
  range_list[node_2].push_back(GetSRange(9.9, 50.0));
  SubTopoGraph sub_topo_graph(range_list);

  const TopoEdge* edge_1_2 = node_1->GetOutEdgeTo(node_2);
  ASSERT_TRUE(edge_1_2 != nullptr);

  std::unordered_set<const TopoEdge*> sub_edges_1_2;
  sub_topo_graph.GetSubInEdgesIntoSubGraph(edge_1_2, &sub_edges_1_2);
  ASSERT_EQ(2, sub_edges_1_2.size());

  for (const auto* edge : sub_edges_1_2) {
    const auto* to_sub_node = edge->ToNode();
    if (to_sub_node->EndS() < 50.0) {
      ASSERT_DOUBLE_EQ(0.0, to_sub_node->StartS());
      ASSERT_DOUBLE_EQ(9.9, to_sub_node->EndS());
    } else {
      ASSERT_DOUBLE_EQ(50.0, to_sub_node->StartS());
      ASSERT_DOUBLE_EQ(100.0, to_sub_node->EndS());
    }
  }
}

TEST(SubTopoGraphTestSuit, one_sub_graph_suc_valid) {
  TopoGraph topo_graph;
  GetTopoGraph(&topo_graph);

  const TopoNode* node_1 = topo_graph.GetNode(TEST_L1);
  ASSERT_TRUE(node_1 != nullptr);
  const TopoNode* node_2 = topo_graph.GetNode(TEST_L2);
  ASSERT_TRUE(node_2 != nullptr);

  std::unordered_map<const TopoNode*, std::vector<NodeSRange>> range_list;
  range_list[node_2].push_back(GetSRange(20.0, TEST_LANE_LENGTH - 5.0));
  SubTopoGraph sub_topo_graph(range_list);

  const TopoEdge* edge_1_2 = node_1->GetOutEdgeTo(node_2);
  ASSERT_TRUE(edge_1_2 != nullptr);

  std::unordered_set<const TopoEdge*> sub_edges_1_2;
  sub_topo_graph.GetSubInEdgesIntoSubGraph(edge_1_2, &sub_edges_1_2);
  ASSERT_EQ(2, sub_edges_1_2.size());

  for (const auto* edge : sub_edges_1_2) {
    const auto* to_sub_node = edge->ToNode();
    if (to_sub_node->EndS() < 50.0) {
      ASSERT_DOUBLE_EQ(0.0, to_sub_node->StartS());
      ASSERT_DOUBLE_EQ(20.0, to_sub_node->EndS());
    } else {
      ASSERT_DOUBLE_EQ(TEST_LANE_LENGTH - 5.0, to_sub_node->StartS());
      ASSERT_DOUBLE_EQ(TEST_LANE_LENGTH, to_sub_node->EndS());
    }
  }
}

TEST(SubTopoGraphTestSuit, two_sub_graph_nearby) {
  TopoGraph topo_graph;
  GetTopoGraph(&topo_graph);

  const TopoNode* node_1 = topo_graph.GetNode(TEST_L1);
  ASSERT_TRUE(node_1 != nullptr);
  const TopoNode* node_2 = topo_graph.GetNode(TEST_L2);
  ASSERT_TRUE(node_2 != nullptr);
  const TopoNode* node_3 = topo_graph.GetNode(TEST_L3);
  ASSERT_TRUE(node_3 != nullptr);
  const TopoNode* node_4 = topo_graph.GetNode(TEST_L4);
  ASSERT_TRUE(node_4 != nullptr);

  std::unordered_map<const TopoNode*, std::vector<NodeSRange>> range_list;
  range_list[node_3].push_back(GetSRange(20.0, 50.0));
  range_list[node_4].push_back(GetSRange(20.0, 50.0));
  SubTopoGraph sub_topo_graph(range_list);

  {
    const TopoEdge* edge_3_4 = node_3->GetOutEdgeTo(node_4);
    ASSERT_TRUE(edge_3_4 != nullptr);

    std::unordered_set<const TopoEdge*> sub_edges_3_4;
    sub_topo_graph.GetSubInEdgesIntoSubGraph(edge_3_4, &sub_edges_3_4);
    // both node has sub nodes, only (none sub node) -> (has sub node)
    // will get sub edges
    ASSERT_EQ(0, sub_edges_3_4.size());
  }

  {
    const TopoEdge* edge_4_3 = node_4->GetOutEdgeTo(node_3);
    ASSERT_TRUE(edge_4_3 != nullptr);

    std::unordered_set<const TopoEdge*> sub_edges_4_3;
    sub_topo_graph.GetSubInEdgesIntoSubGraph(edge_4_3, &sub_edges_4_3);
    // both node has sub nodes, only (none sub node) -> (has sub node)
    // will get sub edges
    ASSERT_EQ(0, sub_edges_4_3.size());
  }

  {
    const TopoEdge* edge_1_3 = node_1->GetOutEdgeTo(node_3);
    ASSERT_TRUE(edge_1_3 != nullptr);

    std::unordered_set<const TopoEdge*> sub_edges_1_3;
    sub_topo_graph.GetSubInEdgesIntoSubGraph(edge_1_3, &sub_edges_1_3);
    ASSERT_EQ(1, sub_edges_1_3.size());

    const auto* edge = *(sub_edges_1_3.begin());
    const auto* to_node = edge->ToNode();
    ASSERT_EQ(2, to_node->InFromAllEdge().size());
    ASSERT_EQ(1, to_node->InFromRightEdge().size());
    ASSERT_EQ(1, to_node->InFromPreEdge().size());
    ASSERT_EQ(1, to_node->OutToAllEdge().size());
    ASSERT_EQ(1, to_node->OutToRightEdge().size());
    ASSERT_EQ(TEST_L3, to_node->LaneId());
    ASSERT_EQ(TEST_R2, to_node->RoadId());
    ASSERT_EQ(node_3, to_node->OriginNode());
    ASSERT_DOUBLE_EQ(0.0, to_node->StartS());
    ASSERT_DOUBLE_EQ(20.0, to_node->EndS());
    ASSERT_DOUBLE_EQ(TEST_LANE_LENGTH, to_node->Length());
    ASSERT_TRUE(to_node->IsSubNode());

    const auto* right_edge = *(to_node->OutToRightEdge().begin());
    const auto* right_to_node = right_edge->ToNode();

    ASSERT_EQ(3, right_to_node->InFromAllEdge().size());
    ASSERT_EQ(1, right_to_node->InFromLeftEdge().size());
    ASSERT_EQ(1, right_to_node->InFromRightEdge().size());
    ASSERT_EQ(1, right_to_node->InFromPreEdge().size());
    ASSERT_EQ(2, right_to_node->OutToAllEdge().size());
    ASSERT_EQ(1, right_to_node->OutToLeftEdge().size());
    ASSERT_EQ(1, right_to_node->OutToRightEdge().size());
    ASSERT_EQ(TEST_L4, right_to_node->LaneId());
    ASSERT_EQ(TEST_R2, right_to_node->RoadId());
    ASSERT_EQ(node_4, right_to_node->OriginNode());
    ASSERT_DOUBLE_EQ(0.0, right_to_node->StartS());
    ASSERT_DOUBLE_EQ(20.0, right_to_node->EndS());
    ASSERT_DOUBLE_EQ(TEST_LANE_LENGTH, right_to_node->Length());
    ASSERT_TRUE(right_to_node->IsSubNode());
  }
}

TEST(SubTopoGraphTestSuit, two_sub_graph_nearby_one_out) {
  TopoGraph topo_graph;
  GetTopoGraph(&topo_graph);

  const TopoNode* node_3 = topo_graph.GetNode(TEST_L3);
  ASSERT_TRUE(node_3 != nullptr);
  const TopoNode* node_4 = topo_graph.GetNode(TEST_L4);
  ASSERT_TRUE(node_4 != nullptr);
  const TopoNode* node_5 = topo_graph.GetNode(TEST_L5);
  ASSERT_TRUE(node_5 != nullptr);
  const TopoNode* node_6 = topo_graph.GetNode(TEST_L6);
  ASSERT_TRUE(node_6 != nullptr);

  std::unordered_map<const TopoNode*, std::vector<NodeSRange>> range_list;
  range_list[node_4].push_back(GetSRange(20.0, 50.0));
  SubTopoGraph sub_topo_graph(range_list);

  {
    const TopoEdge* edge_4_5 = node_4->GetOutEdgeTo(node_5);
    ASSERT_TRUE(edge_4_5 != nullptr);

    std::unordered_set<const TopoEdge*> sub_edges_4_5;
    sub_topo_graph.GetSubInEdgesIntoSubGraph(edge_4_5, &sub_edges_4_5);
    // return origin edge when A has sub node, B has no sub node
    ASSERT_EQ(1, sub_edges_4_5.size());
    const auto* out_edge = *(sub_edges_4_5.begin());
    ASSERT_EQ(edge_4_5, out_edge);
  }

  {
    const TopoEdge* edge_3_4 = node_3->GetOutEdgeTo(node_4);
    ASSERT_TRUE(edge_3_4 != nullptr);

    std::unordered_set<const TopoEdge*> sub_edges_3_4;
    sub_topo_graph.GetSubInEdgesIntoSubGraph(edge_3_4, &sub_edges_3_4);
    // return origin edge when A has sub node, B has no sub node
    ASSERT_EQ(2, sub_edges_3_4.size());
    const auto* one_edge = *(sub_edges_3_4.begin());
    const auto* one_sub_node = one_edge->ToNode();
    const auto* edge_4_sub_5 = one_sub_node->GetOutEdgeTo(node_5);
    ASSERT_TRUE(edge_4_sub_5 != nullptr);
    const auto* right_node = edge_4_sub_5->ToNode();
    ASSERT_EQ(right_node, node_5);

    ASSERT_EQ(1, right_node->InFromAllEdge().size());
    ASSERT_EQ(1, right_node->InFromLeftEdge().size());
    ASSERT_EQ(2, right_node->OutToAllEdge().size());
    ASSERT_EQ(1, right_node->OutToLeftEdge().size());
    ASSERT_EQ(1, right_node->OutToSucEdge().size());
    ASSERT_EQ(TEST_L5, right_node->LaneId());
    ASSERT_EQ(TEST_R2, right_node->RoadId());
    ASSERT_EQ(node_5, right_node->OriginNode());
    ASSERT_DOUBLE_EQ(0.0, right_node->StartS());
    ASSERT_DOUBLE_EQ(TEST_LANE_LENGTH, right_node->EndS());
    ASSERT_DOUBLE_EQ(TEST_LANE_LENGTH, right_node->Length());
    ASSERT_FALSE(right_node->IsSubNode());
  }
}

TEST(SubTopoGraphTestSuit, two_sub_graph_nearby_find_start_node) {
  TopoGraph topo_graph;
  GetTopoGraph(&topo_graph);

  const TopoNode* node_3 = topo_graph.GetNode(TEST_L3);
  ASSERT_TRUE(node_3 != nullptr);

  const TopoNode* node_4 = topo_graph.GetNode(TEST_L4);
  ASSERT_TRUE(node_4 != nullptr);

  std::unordered_map<const TopoNode*, std::vector<NodeSRange>> range_list;
  range_list[node_4].push_back(GetSRange(75.0, 85.0));
  range_list[node_4].push_back(GetSRange(15.0, 30.0));
  range_list[node_4].push_back(GetSRange(45.0, 60.0));
  SubTopoGraph sub_topo_graph(range_list);

  {
    const TopoNode* sub_node = sub_topo_graph.GetSubNodeWithS(node_3, 50.0);
    ASSERT_EQ(sub_node, node_3);
  }

  {
    const TopoNode* sub_node_1 = sub_topo_graph.GetSubNodeWithS(node_4, 25.0);
    ASSERT_TRUE(sub_node_1 == nullptr);
    const TopoNode* sub_node_2 = sub_topo_graph.GetSubNodeWithS(node_4, 55.0);
    ASSERT_TRUE(sub_node_2 == nullptr);
    const TopoNode* sub_node_3 = sub_topo_graph.GetSubNodeWithS(node_4, 80.0);
    ASSERT_TRUE(sub_node_3 == nullptr);
  }

  {
    const TopoNode* sub_node = sub_topo_graph.GetSubNodeWithS(node_4, 10.0);
    ASSERT_TRUE(sub_node != nullptr);
    ASSERT_EQ(3, sub_node->InFromAllEdge().size());
    ASSERT_EQ(1, sub_node->InFromLeftEdge().size());
    ASSERT_EQ(1, sub_node->InFromRightEdge().size());
    ASSERT_EQ(1, sub_node->InFromPreEdge().size());
    ASSERT_EQ(2, sub_node->OutToAllEdge().size());
    ASSERT_EQ(1, sub_node->OutToLeftEdge().size());
    ASSERT_EQ(1, sub_node->OutToRightEdge().size());
    ASSERT_EQ(TEST_L4, sub_node->LaneId());
    ASSERT_EQ(TEST_R2, sub_node->RoadId());
    ASSERT_EQ(node_4, sub_node->OriginNode());
    ASSERT_DOUBLE_EQ(0.0, sub_node->StartS());
    ASSERT_DOUBLE_EQ(15.0, sub_node->EndS());
    ASSERT_DOUBLE_EQ(TEST_LANE_LENGTH, sub_node->Length());
    ASSERT_TRUE(sub_node->IsSubNode());
  }

  {
    const TopoNode* sub_node = sub_topo_graph.GetSubNodeWithS(node_4, 35.0);
    ASSERT_TRUE(sub_node != nullptr);
    ASSERT_EQ(2, sub_node->InFromAllEdge().size());
    ASSERT_EQ(1, sub_node->InFromLeftEdge().size());
    ASSERT_EQ(1, sub_node->InFromRightEdge().size());
    ASSERT_EQ(2, sub_node->OutToAllEdge().size());
    ASSERT_EQ(1, sub_node->OutToLeftEdge().size());
    ASSERT_EQ(1, sub_node->OutToRightEdge().size());
    ASSERT_EQ(TEST_L4, sub_node->LaneId());
    ASSERT_EQ(TEST_R2, sub_node->RoadId());
    ASSERT_EQ(node_4, sub_node->OriginNode());
    ASSERT_DOUBLE_EQ(30.0, sub_node->StartS());
    ASSERT_DOUBLE_EQ(45.0, sub_node->EndS());
    ASSERT_DOUBLE_EQ(TEST_LANE_LENGTH, sub_node->Length());
    ASSERT_TRUE(sub_node->IsSubNode());
  }

  {
    const TopoNode* sub_node = sub_topo_graph.GetSubNodeWithS(node_4, 70.0);
    ASSERT_TRUE(sub_node != nullptr);
    ASSERT_EQ(2, sub_node->InFromAllEdge().size());
    ASSERT_EQ(1, sub_node->InFromLeftEdge().size());
    ASSERT_EQ(1, sub_node->InFromRightEdge().size());
    ASSERT_EQ(2, sub_node->OutToAllEdge().size());
    ASSERT_EQ(1, sub_node->OutToLeftEdge().size());
    ASSERT_EQ(1, sub_node->OutToRightEdge().size());
    ASSERT_EQ(TEST_L4, sub_node->LaneId());
    ASSERT_EQ(TEST_R2, sub_node->RoadId());
    ASSERT_EQ(node_4, sub_node->OriginNode());
    ASSERT_DOUBLE_EQ(60.0, sub_node->StartS());
    ASSERT_DOUBLE_EQ(75.0, sub_node->EndS());
    ASSERT_DOUBLE_EQ(TEST_LANE_LENGTH, sub_node->Length());
    ASSERT_TRUE(sub_node->IsSubNode());
  }

  {
    const TopoNode* sub_node = sub_topo_graph.GetSubNodeWithS(node_4, 90.0);
    ASSERT_TRUE(sub_node != nullptr);
    ASSERT_EQ(2, sub_node->InFromAllEdge().size());
    ASSERT_EQ(1, sub_node->InFromLeftEdge().size());
    ASSERT_EQ(1, sub_node->InFromRightEdge().size());
    ASSERT_EQ(2, sub_node->OutToAllEdge().size());
    ASSERT_EQ(1, sub_node->OutToLeftEdge().size());
    ASSERT_EQ(1, sub_node->OutToRightEdge().size());
    ASSERT_EQ(TEST_L4, sub_node->LaneId());
    ASSERT_EQ(TEST_R2, sub_node->RoadId());
    ASSERT_EQ(node_4, sub_node->OriginNode());
    ASSERT_DOUBLE_EQ(85.0, sub_node->StartS());
    ASSERT_DOUBLE_EQ(TEST_LANE_LENGTH, sub_node->EndS());
    ASSERT_DOUBLE_EQ(TEST_LANE_LENGTH, sub_node->Length());
    ASSERT_TRUE(sub_node->IsSubNode());
  }
}

TEST(SubTopoGraphTestSuit, one_sub_graph_internal_connected) {
  TopoGraph topo_graph;
  GetTopoGraph(&topo_graph);

  const TopoNode* node_1 = topo_graph.GetNode(TEST_L1);
  ASSERT_TRUE(node_1 != nullptr);
  const TopoNode* node_2 = topo_graph.GetNode(TEST_L2);
  ASSERT_TRUE(node_2 != nullptr);

  std::unordered_map<const TopoNode*, std::vector<NodeSRange>> range_list;
  range_list[node_2].push_back(GetSRange(15.9, 15.9));
  range_list[node_2].push_back(GetSRange(50.0, 50.0));
  SubTopoGraph sub_topo_graph(range_list);

  const TopoEdge* edge_1_2 = node_1->GetOutEdgeTo(node_2);
  ASSERT_TRUE(edge_1_2 != nullptr);

  std::unordered_set<const TopoEdge*> sub_edges_1_2;
  sub_topo_graph.GetSubInEdgesIntoSubGraph(edge_1_2, &sub_edges_1_2);
  ASSERT_EQ(3, sub_edges_1_2.size());

  const auto* sub_node_1 = sub_topo_graph.GetSubNodeWithS(node_2, 5.5);
  ASSERT_TRUE(sub_node_1 != nullptr);
  const auto* sub_node_2 = sub_topo_graph.GetSubNodeWithS(node_2, 25.0);
  ASSERT_TRUE(sub_node_2 != nullptr);
  const auto* sub_node_3 = sub_topo_graph.GetSubNodeWithS(node_2, 65.5);
  ASSERT_TRUE(sub_node_3 != nullptr);

  ASSERT_EQ(1, sub_node_1->InFromAllEdge().size());
  ASSERT_EQ(1, sub_node_1->InFromLeftEdge().size());
  ASSERT_EQ(2, sub_node_1->OutToAllEdge().size());
  ASSERT_EQ(1, sub_node_1->OutToLeftEdge().size());
  ASSERT_EQ(1, sub_node_1->OutToSucEdge().size());

  ASSERT_EQ(2, sub_node_2->InFromAllEdge().size());
  ASSERT_EQ(1, sub_node_2->InFromLeftEdge().size());
  ASSERT_EQ(1, sub_node_2->InFromPreEdge().size());
  ASSERT_EQ(2, sub_node_2->OutToAllEdge().size());
  ASSERT_EQ(1, sub_node_2->OutToLeftEdge().size());
  ASSERT_EQ(1, sub_node_2->OutToSucEdge().size());

  ASSERT_EQ(2, sub_node_3->InFromAllEdge().size());
  ASSERT_EQ(1, sub_node_3->InFromLeftEdge().size());
  ASSERT_EQ(1, sub_node_3->InFromPreEdge().size());
  ASSERT_EQ(2, sub_node_3->OutToAllEdge().size());
  ASSERT_EQ(1, sub_node_3->OutToLeftEdge().size());
  ASSERT_EQ(1, sub_node_3->OutToSucEdge().size());
}

TEST(SubTopoGraphTestSuit, one_sub_graph_whole_lane_block) {
  TopoGraph topo_graph;
  GetTopoGraph(&topo_graph);

  const TopoNode* node_1 = topo_graph.GetNode(TEST_L1);
  ASSERT_TRUE(node_1 != nullptr);
  const TopoNode* node_3 = topo_graph.GetNode(TEST_L3);
  ASSERT_TRUE(node_3 != nullptr);

  {
    std::unordered_map<const TopoNode*, std::vector<NodeSRange>> range_list;
    range_list[node_1].push_back(
        GetSRange(TEST_LANE_LENGTH / 2, TEST_LANE_LENGTH / 2));
    range_list[node_3].push_back(GetSRange(0.0, TEST_LANE_LENGTH));
    SubTopoGraph sub_topo_graph(range_list);

    const auto* sub_node_1 =
        sub_topo_graph.GetSubNodeWithS(node_1, TEST_LANE_LENGTH / 2 + 0.1);
    ASSERT_TRUE(sub_node_1 != nullptr);

    ASSERT_EQ(2, sub_node_1->InFromAllEdge().size());
    ASSERT_EQ(1, sub_node_1->InFromRightEdge().size());
    ASSERT_EQ(1, sub_node_1->InFromPreEdge().size());
    ASSERT_EQ(1, sub_node_1->OutToAllEdge().size());
    ASSERT_EQ(1, sub_node_1->OutToRightEdge().size());
  }

  {
    std::unordered_map<const TopoNode*, std::vector<NodeSRange>> range_list;
    range_list[node_1].push_back(
        GetSRange(TEST_LANE_LENGTH / 2, TEST_LANE_LENGTH / 2));
    range_list[node_3].push_back(
        GetSRange(TEST_LANE_LENGTH / 2, TEST_LANE_LENGTH));
    SubTopoGraph sub_topo_graph(range_list);

    const auto* sub_node_1 =
        sub_topo_graph.GetSubNodeWithS(node_1, TEST_LANE_LENGTH / 2 + 0.1);
    ASSERT_TRUE(sub_node_1 != nullptr);

    ASSERT_EQ(2, sub_node_1->InFromAllEdge().size());
    ASSERT_EQ(1, sub_node_1->InFromRightEdge().size());
    ASSERT_EQ(1, sub_node_1->InFromPreEdge().size());
    ASSERT_EQ(2, sub_node_1->OutToAllEdge().size());
    ASSERT_EQ(1, sub_node_1->OutToRightEdge().size());
    ASSERT_EQ(1, sub_node_1->OutToSucEdge().size());

    const auto* sub_node_3 = sub_topo_graph.GetSubNodeWithS(node_3, 0.1);
    ASSERT_TRUE(sub_node_3 != nullptr);

    ASSERT_EQ(2, sub_node_3->InFromAllEdge().size());
    ASSERT_EQ(1, sub_node_3->InFromRightEdge().size());
    ASSERT_EQ(1, sub_node_3->InFromPreEdge().size());
    ASSERT_EQ(1, sub_node_3->OutToAllEdge().size());
    ASSERT_EQ(1, sub_node_3->OutToRightEdge().size());
  }

  {
    std::unordered_map<const TopoNode*, std::vector<NodeSRange>> range_list;
    range_list[node_1].push_back(
        GetSRange(TEST_LANE_LENGTH / 2, TEST_LANE_LENGTH / 2));
    range_list[node_3].push_back(
        GetSRange(TEST_LANE_LENGTH / 4, TEST_LANE_LENGTH / 2));
    range_list[node_3].push_back(
        GetSRange(TEST_LANE_LENGTH * 3 / 4, TEST_LANE_LENGTH));
    SubTopoGraph sub_topo_graph(range_list);

    const auto* sub_node_1 =
        sub_topo_graph.GetSubNodeWithS(node_1, TEST_LANE_LENGTH / 2 + 0.1);
    ASSERT_TRUE(sub_node_1 != nullptr);

    ASSERT_EQ(2, sub_node_1->InFromAllEdge().size());
    ASSERT_EQ(1, sub_node_1->InFromRightEdge().size());
    ASSERT_EQ(1, sub_node_1->InFromPreEdge().size());
    ASSERT_EQ(2, sub_node_1->OutToAllEdge().size());
    ASSERT_EQ(1, sub_node_1->OutToRightEdge().size());
    ASSERT_EQ(1, sub_node_1->OutToSucEdge().size());

    const auto* sub_node_3 =
        sub_topo_graph.GetSubNodeWithS(node_3, TEST_LANE_LENGTH / 2 + 0.1);
    ASSERT_TRUE(sub_node_3 != nullptr);

    ASSERT_EQ(1, sub_node_3->InFromAllEdge().size());
    ASSERT_EQ(1, sub_node_3->InFromRightEdge().size());
    ASSERT_EQ(1, sub_node_3->OutToAllEdge().size());
    ASSERT_EQ(1, sub_node_3->OutToRightEdge().size());
  }
}

}  // namespace routing
}  // namespace apollo
