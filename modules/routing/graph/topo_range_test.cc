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
#include "modules/routing/graph/node_with_range.h"
#include "modules/routing/graph/range_utils.h"

namespace apollo {
namespace routing {

namespace {

void GetRangeVec(std::vector<NodeSRange>* range_vec) {
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

TEST(NodeSRangeTestSuit, basic_test) {
  FLAGS_min_length_for_lane_change = 10.0;
  double start_s_1 = 0.0;
  double start_s_2 = 5.0;
  double end_s_1 = 10.1;
  double end_s_2 = 15.0;
  ASSERT_TRUE(NodeSRange::IsEnoughForChangeLane(start_s_1, end_s_1));
  ASSERT_FALSE(NodeSRange::IsEnoughForChangeLane(start_s_2, end_s_1));

  NodeSRange range_1(end_s_1, start_s_2);
  ASSERT_FALSE(range_1.IsValid());
  ASSERT_FALSE(range_1.IsEnoughForChangeLane());
  ASSERT_DOUBLE_EQ(end_s_1, range_1.StartS());
  ASSERT_DOUBLE_EQ(start_s_2, range_1.EndS());

  NodeSRange range_2(start_s_1, end_s_2);
  ASSERT_TRUE(range_2.IsValid());
  ASSERT_TRUE(range_2.IsEnoughForChangeLane());
  ASSERT_DOUBLE_EQ(start_s_1, range_2.StartS());
  ASSERT_DOUBLE_EQ(end_s_2, range_2.EndS());

  {
    NodeSRange range_from(start_s_1, start_s_2);
    NodeSRange range_to(end_s_1, end_s_2);
    ASSERT_FALSE(range_from.MergeRangeOverlap(range_to));
    ASSERT_TRUE(range_from < range_to);
  }

  {
    NodeSRange range_from(start_s_1, end_s_1);
    NodeSRange range_to(start_s_2, end_s_2);
    ASSERT_TRUE(range_from.MergeRangeOverlap(range_to));
    ASSERT_DOUBLE_EQ(start_s_1, range_from.StartS());
    ASSERT_DOUBLE_EQ(end_s_2, range_from.EndS());
  }
}

TEST(NodeWithRangeTestSuit, basic_test) {
  Node node_1;
  node_1.set_length(20);
  Node node_2;
  node_2.set_length(20);
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
  NodeWithRange node_range_from(tn_1, range_from);
  NodeWithRange node_range_to(tn_2, range_to);

  ASSERT_FALSE(node_range_from < node_range_to);
}

TEST(SortNodeRangeAndSearchTestSuit, start_s_test) {
  std::vector<NodeSRange> range_vec;
  GetRangeVec(&range_vec);

  int index = BinarySearchForStartS(range_vec, 1.2);
  ASSERT_EQ(-1, index);

  index = BinarySearchForStartS(range_vec, 0.0);
  ASSERT_EQ(-1, index);
  index = BinarySearchForStartS(range_vec, 0.9);
  ASSERT_EQ(0, index);
  index = BinarySearchForStartS(range_vec, 1.4);
  ASSERT_EQ(-1, index);
  index = BinarySearchForStartS(range_vec, 1.6);
  ASSERT_EQ(1, index);
  index = BinarySearchForStartS(range_vec, 2.4);
  ASSERT_EQ(-1, index);
  index = BinarySearchForStartS(range_vec, 3.5);
  ASSERT_EQ(2, index);
  index = BinarySearchForStartS(range_vec, 4.4);
  ASSERT_EQ(-1, index);
  index = BinarySearchForStartS(range_vec, 5.8);
  ASSERT_EQ(3, index);
  index = BinarySearchForStartS(range_vec, 6.9);
  ASSERT_EQ(-1, index);
}

TEST(SortNodeRangeAndSearchTestSuit, end_s_test) {
  std::vector<NodeSRange> range_vec;
  GetRangeVec(&range_vec);

  int index = BinarySearchForEndS(range_vec, 1.2);
  ASSERT_EQ(-1, index);

  index = BinarySearchForEndS(range_vec, 0.0);
  ASSERT_EQ(-1, index);
  index = BinarySearchForEndS(range_vec, 0.9);
  ASSERT_EQ(0, index);
  index = BinarySearchForEndS(range_vec, 1.4);
  ASSERT_EQ(-1, index);
  index = BinarySearchForEndS(range_vec, 1.6);
  ASSERT_EQ(1, index);
  index = BinarySearchForEndS(range_vec, 2.4);
  ASSERT_EQ(-1, index);
  index = BinarySearchForEndS(range_vec, 3.5);
  ASSERT_EQ(2, index);
  index = BinarySearchForEndS(range_vec, 4.4);
  ASSERT_EQ(-1, index);
  index = BinarySearchForEndS(range_vec, 5.8);
  ASSERT_EQ(3, index);
  index = BinarySearchForEndS(range_vec, 6.9);
  ASSERT_EQ(-1, index);
}

}  // namespace routing
}  // namespace apollo
