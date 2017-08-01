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

#include "graph/sub_topo_graph.h"
#include "test/test_tool/test_utils.h"

#include "glog/logging.h"

namespace adu {
namespace routing {

namespace {

using ::adu::routing::common::Graph;

NodeWithRange get_range(const TopoNode* topo_node, double start_s, double end_s) {
    NodeWithRange range(NodeSRange(start_s, end_s), topo_node);
    return range;
}

void get_topo_graph(TopoGraph* topo_graph) {
    Graph graph;
    get_graph_2_for_test(&graph);
    ASSERT_TRUE(topo_graph->load_graph(graph));
    ASSERT_EQ(TEST_MAP_VERSION, topo_graph->map_version());
    ASSERT_EQ(TEST_MAP_DISTRICT, topo_graph->map_district());
}

} // namespace

TEST(SubTopoGraphTestSuit, one_sub_graph_all_valid) {
    TopoGraph topo_graph;
    get_topo_graph(&topo_graph);

    const TopoNode* node_1 = topo_graph.get_node(TEST_L1);
    ASSERT_TRUE(node_1 != nullptr);
    const TopoNode* node_2 = topo_graph.get_node(TEST_L2);
    ASSERT_TRUE(node_2 != nullptr);

    std::vector<NodeWithRange> range_list;
    range_list.push_back(get_range(node_2, 20.0, 50.0));
    SubTopoGraph sub_topo_graph(range_list);

    ASSERT_EQ(1, node_1->in_from_all_edge().size());
    ASSERT_EQ(1, node_1->in_from_right_edge().size());
    ASSERT_EQ(2, node_1->out_to_all_edge().size());
    ASSERT_EQ(1, node_1->out_to_suc_edge().size());
    ASSERT_EQ(1, node_1->out_to_right_edge().size());
    ASSERT_DOUBLE_EQ(TEST_LANE_COST, node_1->cost());
    ASSERT_EQ(TEST_L1, node_1->lane_id());
    ASSERT_EQ(TEST_R1, node_1->road_id());
    ASSERT_EQ(node_1, node_1->origin_node());
    ASSERT_DOUBLE_EQ(0.0, node_1->start_s());
    ASSERT_DOUBLE_EQ(TEST_LANE_LENGTH, node_1->end_s());
    ASSERT_DOUBLE_EQ(TEST_LANE_LENGTH, node_1->length());
    ASSERT_FALSE(node_1->is_sub_node());

    const TopoEdge* edge_1_2 = node_1->get_out_edge_to(node_2);
    ASSERT_TRUE(edge_1_2 != nullptr);

    std::unordered_set<const TopoEdge*> sub_edges_1_2;
    sub_topo_graph.get_sub_edges_into_sub_graph(edge_1_2, &sub_edges_1_2);
    ASSERT_EQ(2, sub_edges_1_2.size());

    double min_start_s = std::numeric_limits<double>::max();
    double max_start_s = 0.0;
    double min_end_s = std::numeric_limits<double>::max();
    double max_end_s = 0.0;
    for (const auto* edge : sub_edges_1_2) {
        ASSERT_EQ(node_1, edge->from_node());
        const auto* to_node = edge->to_node();
        min_start_s = std::min(to_node->start_s(), min_start_s);
        max_start_s = std::max(to_node->start_s(), max_start_s);
        min_end_s = std::min(to_node->end_s(), min_end_s);
        max_end_s = std::max(to_node->end_s(), max_end_s);
    }

    ASSERT_DOUBLE_EQ(0.0, min_start_s);
    ASSERT_DOUBLE_EQ(20.0, min_end_s);
    ASSERT_DOUBLE_EQ(50.0, max_start_s);
    ASSERT_DOUBLE_EQ(TEST_LANE_LENGTH, max_end_s);

    const TopoEdge* edge_2_1 = node_2->get_out_edge_to(node_1);
    ASSERT_TRUE(edge_2_1 != nullptr);

    std::unordered_set<const TopoEdge*> sub_edges_2_1;
    sub_topo_graph.get_sub_edges_into_sub_graph(edge_2_1, &sub_edges_2_1);
    // return origin edge when A has sub node, B has no sub node
    ASSERT_EQ(1, sub_edges_2_1.size());
    const auto* out_edge = *(sub_edges_2_1.begin());
    ASSERT_EQ(edge_2_1, out_edge);
}

TEST(SubTopoGraphTestSuit, one_sub_graph_pre_valid) {
    TopoGraph topo_graph;
    get_topo_graph(&topo_graph);

    const TopoNode* node_1 = topo_graph.get_node(TEST_L1);
    ASSERT_TRUE(node_1 != nullptr);
    const TopoNode* node_2 = topo_graph.get_node(TEST_L2);
    ASSERT_TRUE(node_2 != nullptr);

    std::vector<NodeWithRange> range_list;
    range_list.push_back(get_range(node_2, 9.9, 50.0));
    SubTopoGraph sub_topo_graph(range_list);

    const TopoEdge* edge_1_2 = node_1->get_out_edge_to(node_2);
    ASSERT_TRUE(edge_1_2 != nullptr);

    std::unordered_set<const TopoEdge*> sub_edges_1_2;
    sub_topo_graph.get_sub_edges_into_sub_graph(edge_1_2, &sub_edges_1_2);
    ASSERT_EQ(1, sub_edges_1_2.size());

    const auto* edge = *(sub_edges_1_2.begin());
    const auto* to_node = edge->to_node();
    ASSERT_DOUBLE_EQ(50.0, to_node->start_s());
    ASSERT_DOUBLE_EQ(100.0, to_node->end_s());
}

TEST(SubTopoGraphTestSuit, one_sub_graph_suc_valid) {
    TopoGraph topo_graph;
    get_topo_graph(&topo_graph);

    const TopoNode* node_1 = topo_graph.get_node(TEST_L1);
    ASSERT_TRUE(node_1 != nullptr);
    const TopoNode* node_2 = topo_graph.get_node(TEST_L2);
    ASSERT_TRUE(node_2 != nullptr);

    std::vector<NodeWithRange> range_list;
    range_list.push_back(get_range(node_2, 20.0, TEST_LANE_LENGTH - 5.0));
    SubTopoGraph sub_topo_graph(range_list);

    const TopoEdge* edge_1_2 = node_1->get_out_edge_to(node_2);
    ASSERT_TRUE(edge_1_2 != nullptr);

    std::unordered_set<const TopoEdge*> sub_edges_1_2;
    sub_topo_graph.get_sub_edges_into_sub_graph(edge_1_2, &sub_edges_1_2);
    ASSERT_EQ(1, sub_edges_1_2.size());

    const auto* edge = *(sub_edges_1_2.begin());
    const auto* to_node = edge->to_node();
    ASSERT_DOUBLE_EQ(0.0, to_node->start_s());
    ASSERT_DOUBLE_EQ(20.0, to_node->end_s());
}

TEST(SubTopoGraphTestSuit, two_sub_graph_nearby) {
    TopoGraph topo_graph;
    get_topo_graph(&topo_graph);

    const TopoNode* node_1 = topo_graph.get_node(TEST_L1);
    ASSERT_TRUE(node_1 != nullptr);
    const TopoNode* node_2 = topo_graph.get_node(TEST_L2);
    ASSERT_TRUE(node_2 != nullptr);
    const TopoNode* node_3 = topo_graph.get_node(TEST_L3);
    ASSERT_TRUE(node_3 != nullptr);
    const TopoNode* node_4 = topo_graph.get_node(TEST_L4);
    ASSERT_TRUE(node_4 != nullptr);

    std::vector<NodeWithRange> range_list;
    range_list.push_back(get_range(node_3, 20.0, 50.0));
    range_list.push_back(get_range(node_4, 20.0, 50.0));
    SubTopoGraph sub_topo_graph(range_list);

    {
        const TopoEdge* edge_3_4 = node_3->get_out_edge_to(node_4);
        ASSERT_TRUE(edge_3_4 != nullptr);

        std::unordered_set<const TopoEdge*> sub_edges_3_4;
        sub_topo_graph.get_sub_edges_into_sub_graph(edge_3_4, &sub_edges_3_4);
        // both node has sub nodes, only (none sub node) -> (has sub node)
        // will get sub edges
        ASSERT_EQ(0, sub_edges_3_4.size());
    }

    {
        const TopoEdge* edge_4_3 = node_4->get_out_edge_to(node_3);
        ASSERT_TRUE(edge_4_3 != nullptr);

        std::unordered_set<const TopoEdge*> sub_edges_4_3;
        sub_topo_graph.get_sub_edges_into_sub_graph(edge_4_3, &sub_edges_4_3);
        // both node has sub nodes, only (none sub node) -> (has sub node)
        // will get sub edges
        ASSERT_EQ(0, sub_edges_4_3.size());
    }

    {
        const TopoEdge* edge_1_3 = node_1->get_out_edge_to(node_3);
        ASSERT_TRUE(edge_1_3 != nullptr);

        std::unordered_set<const TopoEdge*> sub_edges_1_3;
        sub_topo_graph.get_sub_edges_into_sub_graph(edge_1_3, &sub_edges_1_3);
        ASSERT_EQ(1, sub_edges_1_3.size());

        const auto* edge = *(sub_edges_1_3.begin());
        const auto* to_node = edge->to_node();

        ASSERT_EQ(2, to_node->in_from_all_edge().size());
        ASSERT_EQ(1, to_node->in_from_right_edge().size());
        ASSERT_EQ(1, to_node->in_from_pre_edge().size());
        ASSERT_EQ(1, to_node->out_to_all_edge().size());
        ASSERT_EQ(1, to_node->out_to_right_edge().size());
        ASSERT_EQ(TEST_L3, to_node->lane_id());
        ASSERT_EQ(TEST_R2, to_node->road_id());
        ASSERT_EQ(node_3, to_node->origin_node());
        ASSERT_DOUBLE_EQ(0.0, to_node->start_s());
        ASSERT_DOUBLE_EQ(20.0, to_node->end_s());
        ASSERT_DOUBLE_EQ(TEST_LANE_LENGTH, to_node->length());
        ASSERT_TRUE(to_node->is_sub_node());

        const auto* right_edge = *(to_node->out_to_right_edge().begin());
        const auto* right_to_node = right_edge->to_node();

        ASSERT_EQ(3, right_to_node->in_from_all_edge().size());
        ASSERT_EQ(1, right_to_node->in_from_left_edge().size());
        ASSERT_EQ(1, right_to_node->in_from_right_edge().size());
        ASSERT_EQ(1, right_to_node->in_from_pre_edge().size());
        ASSERT_EQ(2, right_to_node->out_to_all_edge().size());
        ASSERT_EQ(1, right_to_node->out_to_left_edge().size());
        ASSERT_EQ(1, right_to_node->out_to_right_edge().size());
        ASSERT_EQ(TEST_L4, right_to_node->lane_id());
        ASSERT_EQ(TEST_R2, right_to_node->road_id());
        ASSERT_EQ(node_4, right_to_node->origin_node());
        ASSERT_DOUBLE_EQ(0.0, right_to_node->start_s());
        ASSERT_DOUBLE_EQ(20.0, right_to_node->end_s());
        ASSERT_DOUBLE_EQ(TEST_LANE_LENGTH, right_to_node->length());
        ASSERT_TRUE(right_to_node->is_sub_node());
    }
}

TEST(SubTopoGraphTestSuit, two_sub_graph_nearby_one_out) {
    TopoGraph topo_graph;
    get_topo_graph(&topo_graph);

    const TopoNode* node_3 = topo_graph.get_node(TEST_L3);
    ASSERT_TRUE(node_3 != nullptr);
    const TopoNode* node_4 = topo_graph.get_node(TEST_L4);
    ASSERT_TRUE(node_4 != nullptr);
    const TopoNode* node_5 = topo_graph.get_node(TEST_L5);
    ASSERT_TRUE(node_5 != nullptr);
    const TopoNode* node_6 = topo_graph.get_node(TEST_L6);
    ASSERT_TRUE(node_6 != nullptr);

    std::vector<NodeWithRange> range_list;
    range_list.push_back(get_range(node_4, 20.0, 50.0));
    SubTopoGraph sub_topo_graph(range_list);

    {
        const TopoEdge* edge_4_5 = node_4->get_out_edge_to(node_5);
        ASSERT_TRUE(edge_4_5 != nullptr);

        std::unordered_set<const TopoEdge*> sub_edges_4_5;
        sub_topo_graph.get_sub_edges_into_sub_graph(edge_4_5, &sub_edges_4_5);
        // return origin edge when A has sub node, B has no sub node
        ASSERT_EQ(1, sub_edges_4_5.size());
        const auto* out_edge = *(sub_edges_4_5.begin());
        ASSERT_EQ(edge_4_5, out_edge);
    }

    {
        const TopoEdge* edge_3_4 = node_3->get_out_edge_to(node_4);
        ASSERT_TRUE(edge_3_4 != nullptr);

        std::unordered_set<const TopoEdge*> sub_edges_3_4;
        sub_topo_graph.get_sub_edges_into_sub_graph(edge_3_4, &sub_edges_3_4);
        // return origin edge when A has sub node, B has no sub node
        ASSERT_EQ(2, sub_edges_3_4.size());
        const auto* one_edge = *(sub_edges_3_4.begin());
        const auto* one_sub_node = one_edge->to_node();
        const auto* edge_4_sub_5 = one_sub_node->get_out_edge_to(node_5);
        ASSERT_TRUE(edge_4_sub_5 != nullptr);
        const auto* right_node = edge_4_sub_5->to_node();
        ASSERT_EQ(right_node, node_5);

        ASSERT_EQ(1, right_node->in_from_all_edge().size());
        ASSERT_EQ(1, right_node->in_from_left_edge().size());
        ASSERT_EQ(2, right_node->out_to_all_edge().size());
        ASSERT_EQ(1, right_node->out_to_left_edge().size());
        ASSERT_EQ(1, right_node->out_to_suc_edge().size());
        ASSERT_EQ(TEST_L5, right_node->lane_id());
        ASSERT_EQ(TEST_R2, right_node->road_id());
        ASSERT_EQ(node_5, right_node->origin_node());
        ASSERT_DOUBLE_EQ(0.0, right_node->start_s());
        ASSERT_DOUBLE_EQ(TEST_LANE_LENGTH, right_node->end_s());
        ASSERT_DOUBLE_EQ(TEST_LANE_LENGTH, right_node->length());
        ASSERT_FALSE(right_node->is_sub_node());
    }
}

TEST(SubTopoGraphTestSuit, two_sub_graph_nearby_find_start_node) {
    TopoGraph topo_graph;
    get_topo_graph(&topo_graph);

    const TopoNode* node_3 = topo_graph.get_node(TEST_L3);
    ASSERT_TRUE(node_3 != nullptr);

    const TopoNode* node_4 = topo_graph.get_node(TEST_L4);
    ASSERT_TRUE(node_4 != nullptr);

    std::vector<NodeWithRange> range_list;
    range_list.push_back(get_range(node_4, 75.0, 85.0));
    range_list.push_back(get_range(node_4, 15.0, 30.0));
    range_list.push_back(get_range(node_4, 45.0, 60.0));
    SubTopoGraph sub_topo_graph(range_list);

    {
        const TopoNode* sub_node = sub_topo_graph.get_sub_node_with_s(node_3, 50.0);
        ASSERT_EQ(sub_node, node_3);
    }

    {
        const TopoNode* sub_node_1 = sub_topo_graph.get_sub_node_with_s(node_4, 25.0);
        ASSERT_TRUE(sub_node_1 == nullptr);
        const TopoNode* sub_node_2 = sub_topo_graph.get_sub_node_with_s(node_4, 55.0);
        ASSERT_TRUE(sub_node_2 == nullptr);
        const TopoNode* sub_node_3 = sub_topo_graph.get_sub_node_with_s(node_4, 80.0);
        ASSERT_TRUE(sub_node_3 == nullptr);
    }

    {
        const TopoNode* sub_node = sub_topo_graph.get_sub_node_with_s(node_4, 10.0);
        ASSERT_TRUE(sub_node != nullptr);
        ASSERT_EQ(3, sub_node->in_from_all_edge().size());
        ASSERT_EQ(1, sub_node->in_from_left_edge().size());
        ASSERT_EQ(1, sub_node->in_from_right_edge().size());
        ASSERT_EQ(1, sub_node->in_from_pre_edge().size());
        ASSERT_EQ(2, sub_node->out_to_all_edge().size());
        ASSERT_EQ(1, sub_node->out_to_left_edge().size());
        ASSERT_EQ(1, sub_node->out_to_right_edge().size());
        ASSERT_EQ(TEST_L4, sub_node->lane_id());
        ASSERT_EQ(TEST_R2, sub_node->road_id());
        ASSERT_EQ(node_4, sub_node->origin_node());
        ASSERT_DOUBLE_EQ(0.0, sub_node->start_s());
        ASSERT_DOUBLE_EQ(15.0, sub_node->end_s());
        ASSERT_DOUBLE_EQ(TEST_LANE_LENGTH, sub_node->length());
        ASSERT_TRUE(sub_node->is_sub_node());
    }

    {
        const TopoNode* sub_node = sub_topo_graph.get_sub_node_with_s(node_4, 35.0);
        ASSERT_TRUE(sub_node != nullptr);
        ASSERT_EQ(2, sub_node->in_from_all_edge().size());
        ASSERT_EQ(1, sub_node->in_from_left_edge().size());
        ASSERT_EQ(1, sub_node->in_from_right_edge().size());
        ASSERT_EQ(2, sub_node->out_to_all_edge().size());
        ASSERT_EQ(1, sub_node->out_to_left_edge().size());
        ASSERT_EQ(1, sub_node->out_to_right_edge().size());
        ASSERT_EQ(TEST_L4, sub_node->lane_id());
        ASSERT_EQ(TEST_R2, sub_node->road_id());
        ASSERT_EQ(node_4, sub_node->origin_node());
        ASSERT_DOUBLE_EQ(30.0, sub_node->start_s());
        ASSERT_DOUBLE_EQ(45.0, sub_node->end_s());
        ASSERT_DOUBLE_EQ(TEST_LANE_LENGTH, sub_node->length());
        ASSERT_TRUE(sub_node->is_sub_node());
    }

    {
        const TopoNode* sub_node = sub_topo_graph.get_sub_node_with_s(node_4, 70.0);
        ASSERT_TRUE(sub_node != nullptr);
        ASSERT_EQ(2, sub_node->in_from_all_edge().size());
        ASSERT_EQ(1, sub_node->in_from_left_edge().size());
        ASSERT_EQ(1, sub_node->in_from_right_edge().size());
        ASSERT_EQ(2, sub_node->out_to_all_edge().size());
        ASSERT_EQ(1, sub_node->out_to_left_edge().size());
        ASSERT_EQ(1, sub_node->out_to_right_edge().size());
        ASSERT_EQ(TEST_L4, sub_node->lane_id());
        ASSERT_EQ(TEST_R2, sub_node->road_id());
        ASSERT_EQ(node_4, sub_node->origin_node());
        ASSERT_DOUBLE_EQ(60.0, sub_node->start_s());
        ASSERT_DOUBLE_EQ(75.0, sub_node->end_s());
        ASSERT_DOUBLE_EQ(TEST_LANE_LENGTH, sub_node->length());
        ASSERT_TRUE(sub_node->is_sub_node());
    }

    {
        const TopoNode* sub_node = sub_topo_graph.get_sub_node_with_s(node_4, 90.0);
        ASSERT_TRUE(sub_node != nullptr);
        ASSERT_EQ(2, sub_node->in_from_all_edge().size());
        ASSERT_EQ(1, sub_node->in_from_left_edge().size());
        ASSERT_EQ(1, sub_node->in_from_right_edge().size());
        ASSERT_EQ(2, sub_node->out_to_all_edge().size());
        ASSERT_EQ(1, sub_node->out_to_left_edge().size());
        ASSERT_EQ(1, sub_node->out_to_right_edge().size());
        ASSERT_EQ(TEST_L4, sub_node->lane_id());
        ASSERT_EQ(TEST_R2, sub_node->road_id());
        ASSERT_EQ(node_4, sub_node->origin_node());
        ASSERT_DOUBLE_EQ(85.0, sub_node->start_s());
        ASSERT_DOUBLE_EQ(TEST_LANE_LENGTH, sub_node->end_s());
        ASSERT_DOUBLE_EQ(TEST_LANE_LENGTH, sub_node->length());
        ASSERT_TRUE(sub_node->is_sub_node());
    }
}

}  // namespace routing
}  // namespace adu

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


