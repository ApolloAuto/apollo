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

#include "graph/topo_range.h"
#include "graph/topo_node.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

#include "topo_graph.pb.h"

namespace adu {
namespace routing {

namespace {

using ::adu::routing::common::Node;

void get_range_vec(std::vector<NodeSRange>* range_vec) {
    double s_s_1 = 0.1;
    double e_s_1 = 1.1;
    double s_s_2 = 1.5;
    double e_s_2 = 2.0;
    double s_s_3 = 3.0;
    double e_s_3 = 4.1;
    double s_s_4 = 5.5;
    double e_s_4 = 6.0;

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

TEST(NodeSRangeTestSuit, basic_test) {
    double start_s_1 = 0.0;
    double start_s_2 = 5.0;
    double end_s_1 = 10.1;
    double end_s_2 = 15.0;
    ASSERT_TRUE(NodeSRange::is_enough_for_change_lane(start_s_1, end_s_1));
    ASSERT_FALSE(NodeSRange::is_enough_for_change_lane(start_s_2, end_s_1));

    NodeSRange range_1(end_s_1, start_s_2);
    ASSERT_FALSE(range_1.is_valid());
    ASSERT_FALSE(range_1.is_enough_for_change_lane());
    ASSERT_DOUBLE_EQ(end_s_1, range_1.start_s());
    ASSERT_DOUBLE_EQ(start_s_2, range_1.end_s());

    NodeSRange range_2(start_s_1, end_s_2);
    ASSERT_TRUE(range_2.is_valid());
    ASSERT_TRUE(range_2.is_enough_for_change_lane());
    ASSERT_DOUBLE_EQ(start_s_1, range_2.start_s());
    ASSERT_DOUBLE_EQ(end_s_2, range_2.end_s());

    {
        NodeSRange range_from(start_s_1, start_s_2);
        NodeSRange range_to(end_s_1, end_s_2);
        ASSERT_FALSE(range_from.merge_range_overlap(range_to));
        ASSERT_TRUE(range_from < range_to);
    }

    {
        NodeSRange range_from(start_s_1, end_s_1);
        NodeSRange range_to(start_s_2, end_s_2);
        ASSERT_TRUE(range_from.merge_range_overlap(range_to));
        ASSERT_DOUBLE_EQ(start_s_1, range_from.start_s());
        ASSERT_DOUBLE_EQ(end_s_2, range_from.end_s());
    }
}

TEST(NodeWithRangeTestSuit, basic_test) {
    Node node_1;
    Node node_2;
    TopoNode topo_node_1(node_1);
    TopoNode topo_node_2(node_2);
    double start_s_1 = 0.0;
    double start_s_2 = 5.0;
    double end_s_1 = 10.1;
    double end_s_2 = 15.0;
    NodeSRange range_from(start_s_1, start_s_2);
    NodeSRange range_to(end_s_1, end_s_2);

    const TopoNode* tn_1 = &topo_node_1;
    const TopoNode* tn_2 = &topo_node_2;
    NodeWithRange node_range_from(range_from, tn_1);
    NodeWithRange node_range_to(range_to, tn_2);

    ASSERT_FALSE(node_range_from < node_range_to);
}

TEST(SortNodeRangeAndSearchTestSuit, start_s_test) {
    std::vector<NodeSRange> range_vec;
    get_range_vec(&range_vec);

    int index = binary_search_for_start_s(range_vec, 1.2);
    ASSERT_EQ(-1, index);

    index = binary_search_for_start_s(range_vec, 0.0);
    ASSERT_EQ(-1, index);
    index = binary_search_for_start_s(range_vec, 0.9);
    ASSERT_EQ(0, index);
    index = binary_search_for_start_s(range_vec, 1.4);
    ASSERT_EQ(-1, index);
    index = binary_search_for_start_s(range_vec, 1.6);
    ASSERT_EQ(1, index);
    index = binary_search_for_start_s(range_vec, 2.4);
    ASSERT_EQ(-1, index);
    index = binary_search_for_start_s(range_vec, 3.5);
    ASSERT_EQ(2, index);
    index = binary_search_for_start_s(range_vec, 4.4);
    ASSERT_EQ(-1, index);
    index = binary_search_for_start_s(range_vec, 5.8);
    ASSERT_EQ(3, index);
    index = binary_search_for_start_s(range_vec, 6.9);
    ASSERT_EQ(-1, index);
}

TEST(SortNodeRangeAndSearchTestSuit, end_s_test) {
    std::vector<NodeSRange> range_vec;
    get_range_vec(&range_vec);

    int index = binary_search_for_end_s(range_vec, 1.2);
    ASSERT_EQ(-1, index);

    index = binary_search_for_end_s(range_vec, 0.0);
    ASSERT_EQ(-1, index);
    index = binary_search_for_end_s(range_vec, 0.9);
    ASSERT_EQ(0, index);
    index = binary_search_for_end_s(range_vec, 1.4);
    ASSERT_EQ(-1, index);
    index = binary_search_for_end_s(range_vec, 1.6);
    ASSERT_EQ(1, index);
    index = binary_search_for_end_s(range_vec, 2.4);
    ASSERT_EQ(-1, index);
    index = binary_search_for_end_s(range_vec, 3.5);
    ASSERT_EQ(2, index);
    index = binary_search_for_end_s(range_vec, 4.4);
    ASSERT_EQ(-1, index);
    index = binary_search_for_end_s(range_vec, 5.8);
    ASSERT_EQ(3, index);
    index = binary_search_for_end_s(range_vec, 6.9);
    ASSERT_EQ(-1, index);
}

}  // namespace routing
}  // namespace adu

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

