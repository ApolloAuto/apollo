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

#include "modules/perception/common/graph_util.h"

#include "Eigen/Core"
#include "gtest/gtest.h"

#include "modules/common/log.h"

namespace apollo {
namespace perception {

class GraphUtilTest : public testing::Test {
 protected:
  GraphUtilTest() = default;
  virtual ~GraphUtilTest() = default;
  void SetUp() {}
  void TearDown() {}
};

TEST_F(GraphUtilTest, ConnectedComponentAnalysis) {
  Eigen::MatrixXf association_mat(3, 4);
  association_mat << 0.3, 1.2, 4.0, 3.0, 0.9, 2.0, 3.0, 8.0, 4.0, 3.0, 0.3, 0.1;
  const float connected_threshold = 2.1;
  // Compute connected components within given threshold
  int no_track = association_mat.rows();
  int no_obj = association_mat.cols();
  std::vector<std::vector<int>> nb_graph;
  nb_graph.resize(no_track + no_obj);
  for (int i = 0; i < no_track; i++) {
    for (int j = 0; j < no_obj; j++) {
      if (association_mat(i, j) <= connected_threshold) {
        nb_graph[i].push_back(no_track + j);
        nb_graph[j + no_track].push_back(i);
      }
    }
  }

  std::vector<std::vector<int>> components;
  ConnectedComponentAnalysis(nb_graph, &components);
  EXPECT_EQ(2, components.size());
  EXPECT_EQ(4, components[0].size());
  EXPECT_EQ(0, components[0][0]);
  EXPECT_EQ(3, components[0][1]);
  EXPECT_EQ(4, components[0][2]);
  EXPECT_EQ(1, components[0][3]);
  EXPECT_EQ(3, components[1].size());
  EXPECT_EQ(2, components[1][0]);
  EXPECT_EQ(5, components[1][1]);
  EXPECT_EQ(6, components[1][2]);
}

}  // namespace perception
}  // namespace apollo
