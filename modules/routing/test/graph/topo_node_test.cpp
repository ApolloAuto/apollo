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

#include "test/test_tool/test_utils.h"

namespace adu {
namespace routing {

namespace {

using ::adu::routing::common::Node;

void get_range_vec(std::vector<NodeSRange>* range_vec) {
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

    ASSERT_DOUBLE_EQ(s_s_1, range_vec->at(0).start_s());
    ASSERT_DOUBLE_EQ(e_s_1, range_vec->at(0).end_s());
    ASSERT_DOUBLE_EQ(s_s_2, range_vec->at(1).start_s());
    ASSERT_DOUBLE_EQ(e_s_2, range_vec->at(1).end_s());
    ASSERT_DOUBLE_EQ(s_s_3, range_vec->at(2).start_s());
    ASSERT_DOUBLE_EQ(e_s_3, range_vec->at(2).end_s());
    ASSERT_DOUBLE_EQ(s_s_4, range_vec->at(3).start_s());
    ASSERT_DOUBLE_EQ(e_s_4, range_vec->at(3).end_s());
}

} // namespace

TEST(TopoNodeTestSuit, static_func_test) {
    std::vector<NodeSRange> range_vec;
    get_range_vec(&range_vec);
    {
        double start_s = 13.0;
        double end_s = 26.0;
        ASSERT_TRUE(TopoNode::is_out_range_enough(range_vec, start_s, end_s));
    }
    {
        double start_s = 13.0;
        double end_s = 23.0;
        ASSERT_FALSE(TopoNode::is_out_range_enough(range_vec, start_s, end_s));
    }
    {
        double start_s = 22.0;
        double end_s = 32.0;
        ASSERT_FALSE(TopoNode::is_out_range_enough(range_vec, start_s, end_s));
    }
    {
        double start_s = 31.0;
        double end_s = 44.0;
        ASSERT_TRUE(TopoNode::is_out_range_enough(range_vec, start_s, end_s));
    }
    {
        double start_s = -10;
        double end_s = 100;
        ASSERT_TRUE(TopoNode::is_out_range_enough(range_vec, start_s, end_s));
    }
}

TEST(TopoNodeTestSuit, basic_test) {
    Node node;
    get_node_detail_for_test(&node, TEST_L1, TEST_R1);
    TopoNode topo_node(node);
    ASSERT_EQ(node.DebugString(), topo_node.node().DebugString());
    ASSERT_EQ(TEST_L1, topo_node.lane_id());
    ASSERT_EQ(TEST_R1, topo_node.road_id());
    ASSERT_DOUBLE_EQ(TEST_MIDDLE_S, topo_node.anchor_point().x());
    ASSERT_DOUBLE_EQ(0.0, topo_node.anchor_point().y());
    ASSERT_DOUBLE_EQ(TEST_LANE_LENGTH, topo_node.length());
    ASSERT_DOUBLE_EQ(TEST_LANE_COST, topo_node.cost());
    ASSERT_TRUE(topo_node.is_virtual());
}

}  // namespace routing
}  // namespace adu

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

