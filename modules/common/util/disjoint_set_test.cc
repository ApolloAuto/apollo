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

#include "modules/common/util/disjoint_set.h"

#include "cyber/common/log.h"
#include "gtest/gtest.h"

namespace apollo {
namespace common {
namespace util {

class DisjointSetTest : public testing::Test {
 protected:
  DisjointSetTest() {
    node1_ = new Node();
    node2_ = new Node();
    node3_ = new Node();
  }
  virtual ~DisjointSetTest() {
    if (node1_ != nullptr) {
      delete node1_;
      node1_ = nullptr;
    }
    if (node2_ != nullptr) {
      delete node2_;
      node2_ = nullptr;
    }
    if (node3_ != nullptr) {
      delete node3_;
      node3_ = nullptr;
    }
  }
  void SetUp() {
    DisjointSetMakeSet(node1_);
    node2_->parent = node1_;
    node3_->parent = node2_;
  }
  void TearDown() {}
  struct Node {
    Node* parent = nullptr;
    char node_rank = 0;
  };
  Node* node1_ = nullptr;
  Node* node2_ = nullptr;
  Node* node3_ = nullptr;
};

TEST_F(DisjointSetTest, DisjointSetMakeSet) {
  Node* node = new Node();
  DisjointSetMakeSet<Node>(node);
  EXPECT_EQ(node, node->parent);
  EXPECT_EQ(0, node->node_rank);
  if (node != nullptr) {
    delete node;
    node = nullptr;
  }
}

TEST_F(DisjointSetTest, DisjointSetFindRecursive) {
  Node* root_node2 = DisjointSetFindRecursive<Node>(node2_);
  EXPECT_EQ(node1_, root_node2);
  Node* root_node3 = DisjointSetFindRecursive<Node>(node3_);
  EXPECT_EQ(node1_, root_node3);
}

TEST_F(DisjointSetTest, DisjointSetFind) {
  Node* root_node2 = DisjointSetFind<Node>(node2_);
  EXPECT_EQ(node1_, root_node2);
  Node* root_node3 = DisjointSetFind<Node>(node3_);
  EXPECT_EQ(node1_, root_node3);
  EXPECT_EQ(node1_, root_node3->parent);
}

TEST_F(DisjointSetTest, DisjointSetMerge) {
  DisjointSetMerge<Node>(node1_, node2_);
}

TEST_F(DisjointSetTest, DisjointSetUnion) {
  DisjointSetUnion<Node>(node1_, node2_);
  EXPECT_EQ(node1_->parent, node2_->parent);
}

}  // namespace util
}  // namespace common
}  // namespace apollo
