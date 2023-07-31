/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/common/algorithm/graph/graph_segmentor.h"

#include "gtest/gtest.h"

namespace apollo {
namespace perception {
namespace algorithm {

class GraphSegmentorTest : public testing::Test {
 protected:
  void SetUp() {
    edges_ = new Edge[10];
    edges_[0].w = 6.f;
    edges_[0].a = 1;
    edges_[0].b = 2;
    edges_[1].w = 1.f;
    edges_[1].a = 1;
    edges_[1].b = 3;
    edges_[2].w = 5.f;
    edges_[2].a = 1;
    edges_[2].b = 4;
    edges_[3].w = 5.f;
    edges_[3].a = 3;
    edges_[3].b = 2;
    edges_[4].w = 5.f;
    edges_[4].a = 3;
    edges_[4].b = 4;
    edges_[5].w = 3.f;
    edges_[5].a = 5;
    edges_[5].b = 2;
    edges_[6].w = 6.f;
    edges_[6].a = 3;
    edges_[6].b = 5;
    edges_[7].w = 4.f;
    edges_[7].a = 3;
    edges_[7].b = 0;
    edges_[8].w = 2.f;
    edges_[8].a = 4;
    edges_[8].b = 0;
    edges_[9].w = 6.f;
    edges_[9].a = 5;
    edges_[9].b = 0;
  }

  void TearDown() {
    delete[] edges_;
    edges_ = nullptr;
  }

  Edge* edges_;
  const int num_edges_ = 10;
  const int num_vertices_ = 6;
};

TEST_F(GraphSegmentorTest, test_edge_comparison) {
  EXPECT_LT(edges_[1], edges_[0]);
  EXPECT_FALSE(edges_[3] < edges_[4]);
  EXPECT_FALSE(edges_[6] < edges_[7]);
}

TEST_F(GraphSegmentorTest, test_segment_graph) {
  {
    GraphSegmentor segmentor;
    segmentor.Init(5.0);
    segmentor.SegmentGraph(num_vertices_, num_edges_, nullptr, false);
    EXPECT_EQ(0, segmentor.universe().GetSetsNum());
    segmentor.SegmentGraph(num_vertices_, num_edges_, edges_, true);
    EXPECT_EQ(3, segmentor.universe().GetSetsNum());
  }
  {
    GraphSegmentor segmentor;
    segmentor.Init(6.0);
    segmentor.SegmentGraph(num_vertices_, num_edges_, edges_);
    EXPECT_EQ(1, segmentor.universe().GetSetsNum());
  }
  {
    GraphSegmentor segmentor;
    segmentor.Init(2.0);
    segmentor.SegmentGraph(num_vertices_, num_edges_, edges_);
    EXPECT_EQ(4, segmentor.universe().GetSetsNum());
  }
}

}  // namespace algorithm
}  // namespace perception
}  // namespace apollo
