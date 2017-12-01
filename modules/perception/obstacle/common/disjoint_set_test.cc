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

#include "modules/perception/obstacle/common/disjoint_set.h"

#include "gtest/gtest.h"
#include "modules/common/log.h"

namespace apollo {
namespace perception {

class DisjointSetTest : public testing::Test {
 protected:
  DisjointSetTest() {
    _node1 = new Node();
    _node2 = new Node();
    _node3 = new Node();
  }
  virtual ~DisjointSetTest() {
    if (_node1 != nullptr) {
      delete _node1;
      _node1 = nullptr;
    }
    if (_node2 != nullptr) {
      delete _node2;
      _node2 = nullptr;
    }
    if (_node3 != nullptr) {
      delete _node3;
      _node3 = nullptr;
    }
  }
  void SetUp() {
    DisjointSetMakeSet(_node1);
    _node2->parent = _node1;
    _node3->parent = _node2;
  }
  void TearDown() {}
  struct Node {
    Node* parent;
    char node_rank;

    Node() {
      parent = nullptr;
      node_rank = 0;
    }
  };
  Node* _node1;
  Node* _node2;
  Node* _node3;
};

TEST_F(DisjointSetTest, DisjointSetMakeSet) {
  Node* node = new Node();
  ;
  DisjointSetMakeSet<Node>(node);
  EXPECT_EQ(node, node->parent);
  EXPECT_EQ(0, node->node_rank);
  if (node != nullptr) {
    delete node;
    node = nullptr;
  }
}

TEST_F(DisjointSetTest, DisjointSetFindRecursive) {
  Node* root_node2 = DisjointSetFindRecursive<Node>(_node2);
  EXPECT_EQ(_node1, root_node2);
  Node* root_node3 = DisjointSetFindRecursive<Node>(_node3);
  EXPECT_EQ(_node1, root_node3);
}

TEST_F(DisjointSetTest, DisjointSetFind) {
  Node* root_node2 = DisjointSetFind<Node>(_node2);
  EXPECT_EQ(_node1, root_node2);
  Node* root_node3 = DisjointSetFind<Node>(_node3);
  EXPECT_EQ(_node1, root_node3);
  EXPECT_EQ(_node1, root_node3->parent);
}

TEST_F(DisjointSetTest, DisjointSetMerge) {
  DisjointSetMerge<Node>(_node1, _node2);
}

TEST_F(DisjointSetTest, DisjointSetUnion) {
  DisjointSetUnion<Node>(_node1, _node2);
  EXPECT_EQ(_node1->parent, _node2->parent);
}

}  // namespace perception
}  // namespace apollo
