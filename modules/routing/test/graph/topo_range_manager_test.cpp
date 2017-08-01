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

#include "graph/topo_range_manager.h"
#include "test/test_tool/test_utils.h"

namespace adu {
namespace routing {

using ::adu::routing::common::Graph;

TEST(TopoRangeManagerTestSuit, test_graph_1) {
    Graph graph;
    get_graph_2_for_test(&graph);

    TopoGraph topo_graph;
    ASSERT_TRUE(topo_graph.load_graph(graph));

    const auto* node_1 = topo_graph.get_node(TEST_L1);
    ASSERT_TRUE(node_1 != nullptr);
    const auto* node_2 = topo_graph.get_node(TEST_L2);
    ASSERT_TRUE(node_2 != nullptr);
    const auto* node_3 = topo_graph.get_node(TEST_L3);
    ASSERT_TRUE(node_3 != nullptr);
    const auto* node_4 = topo_graph.get_node(TEST_L4);
    ASSERT_TRUE(node_4 != nullptr);
    const auto* node_5 = topo_graph.get_node(TEST_L5);
    ASSERT_TRUE(node_5 != nullptr);
    const auto* node_6 = topo_graph.get_node(TEST_L6);
    ASSERT_TRUE(node_6 != nullptr);

    double start_node_s = 10.0;
    double end_node_s = 50.0;

    NodeSRangeManager manager;
    manager.init_node_range(start_node_s, end_node_s, node_1, node_3);

    NodeSRange range_s = manager.get_node_range(node_1);
    ASSERT_EQ(start_node_s, range_s.start_s());
    ASSERT_EQ(TEST_LANE_LENGTH, range_s.end_s());

    NodeSRange range_e = manager.get_node_range(node_3);
    ASSERT_EQ(0.0, range_e.start_s());
    ASSERT_EQ(end_node_s, range_e.end_s());

    ASSERT_EQ(start_node_s, manager.get_node_start_s(node_2));
    ASSERT_EQ(TEST_LANE_LENGTH, manager.get_node_end_s(node_2));
    ASSERT_EQ(0.0, manager.get_node_start_s(node_4));
    ASSERT_EQ(end_node_s, manager.get_node_end_s(node_4));
    ASSERT_EQ(0.0, manager.get_node_start_s(node_5));
    ASSERT_EQ(end_node_s, manager.get_node_end_s(node_5));
    ASSERT_EQ(0.0, manager.get_node_start_s(node_6));
    ASSERT_EQ(TEST_LANE_LENGTH, manager.get_node_end_s(node_6));
}

TEST(TopoRangeManagerTestSuit, test_graph_2) {
    Graph graph;
    get_graph_2_for_test(&graph);

    TopoGraph topo_graph;
    ASSERT_TRUE(topo_graph.load_graph(graph));

    const auto* node_1 = topo_graph.get_node(TEST_L1);
    ASSERT_TRUE(node_1 != nullptr);
    const auto* node_2 = topo_graph.get_node(TEST_L2);
    ASSERT_TRUE(node_2 != nullptr);
    const auto* node_3 = topo_graph.get_node(TEST_L3);
    ASSERT_TRUE(node_3 != nullptr);
    const auto* node_4 = topo_graph.get_node(TEST_L4);
    ASSERT_TRUE(node_4 != nullptr);
    const auto* node_5 = topo_graph.get_node(TEST_L5);
    ASSERT_TRUE(node_5 != nullptr);
    const auto* node_6 = topo_graph.get_node(TEST_L6);
    ASSERT_TRUE(node_6 != nullptr);

    double start_node_s = 10.0;
    double end_node_s = 50.0;
    
    {
        NodeSRangeManager manager;
        manager.init_node_range(start_node_s, end_node_s, node_1, node_1);
        NodeSRange range_s = manager.get_node_range(node_1);
        ASSERT_EQ(start_node_s, range_s.start_s());
        ASSERT_EQ(end_node_s, range_s.end_s());
    }

    {
        NodeSRangeManager manager;
        manager.init_node_range(start_node_s, end_node_s, node_3, node_4);
        NodeSRange range_s = manager.get_node_range(node_3);
        ASSERT_EQ(start_node_s, range_s.start_s());
        ASSERT_EQ(end_node_s, range_s.end_s());
        range_s = manager.get_node_range(node_4);
        ASSERT_EQ(start_node_s, range_s.start_s());
        ASSERT_EQ(end_node_s, range_s.end_s());
    }

    {
        NodeSRangeManager manager;
        manager.init_node_range(start_node_s, end_node_s, node_3, node_5);
        NodeSRange range_s = manager.get_node_range(node_3);
        ASSERT_EQ(start_node_s, range_s.start_s());
        ASSERT_EQ(end_node_s, range_s.end_s());
        range_s = manager.get_node_range(node_4);
        ASSERT_EQ(start_node_s, range_s.start_s());
        ASSERT_EQ(end_node_s, range_s.end_s());
        range_s = manager.get_node_range(node_5);
        ASSERT_EQ(start_node_s, range_s.start_s());
        ASSERT_EQ(end_node_s, range_s.end_s());
    }
}


}  // namespace routing
}  // namespace adu

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


