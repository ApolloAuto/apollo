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

#include "test/test_tool/test_utils.h"

namespace adu {
namespace routing {

using ::adu::routing::common::Graph;

TEST(TopoGraphTestSuit, test_graph_1) {
    Graph graph;
    get_graph_for_test(&graph);

    TopoGraph topo_graph;
    ASSERT_TRUE(topo_graph.load_graph(graph));

    ASSERT_EQ(TEST_MAP_VERSION, topo_graph.map_version());
    ASSERT_EQ(TEST_MAP_DISTRICT, topo_graph.map_district());

    const TopoNode* node_1 = topo_graph.get_node(TEST_L1);
    ASSERT_TRUE(node_1 != nullptr);
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

    const TopoNode* node_4 = topo_graph.get_node(TEST_L4);
    ASSERT_TRUE(node_4 != nullptr);
    ASSERT_EQ(2, node_4->in_from_all_edge().size());
    ASSERT_EQ(1, node_4->in_from_pre_edge().size());
    ASSERT_EQ(1, node_4->in_from_left_edge().size());
    ASSERT_EQ(1, node_4->out_to_all_edge().size());
    ASSERT_EQ(1, node_4->out_to_left_edge().size());
    ASSERT_DOUBLE_EQ(TEST_LANE_COST, node_4->cost());
    ASSERT_EQ(TEST_L4, node_4->lane_id());
    ASSERT_EQ(TEST_R2, node_4->road_id());
    ASSERT_EQ(node_4, node_4->origin_node());
    ASSERT_DOUBLE_EQ(0.0, node_4->start_s());
    ASSERT_DOUBLE_EQ(TEST_LANE_LENGTH, node_4->end_s());
    ASSERT_DOUBLE_EQ(TEST_LANE_LENGTH, node_4->length());
    ASSERT_FALSE(node_4->is_sub_node());
}

}  // namespace routing
}  // namespace adu

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


