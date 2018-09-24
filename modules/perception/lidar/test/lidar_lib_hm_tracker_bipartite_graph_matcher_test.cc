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
#include <gtest/gtest.h>
#include "modules/perception/base/log.h"
#include "modules/perception/common/graph/secure_matrix.h"
#include "modules/perception/lidar/lib/tracker/hm_tracker/gnn_bipartite_graph_matcher.h"
#include "modules/perception/lidar/lib/tracker/hm_tracker/multi_hm_bipartite_graph_matcher.h"

namespace apollo {
namespace perception {
namespace lidar {

TEST(BipartiteGraphMatcherTest, multi_hm_bipartite_graph_matcher_test) {
  MultiHmBipartiteGraphMatcher *matcher = new MultiHmBipartiteGraphMatcher();
  common::SecureMat<float> *cost_mat = matcher->cost_matrix();
  cost_mat->reserve(10, 10);
  cost_mat->resize(3, 4);
  (*cost_mat)(0, 0) = 0.2;
  (*cost_mat)(0, 1) = 1.2;
  (*cost_mat)(0, 2) = 4;
  (*cost_mat)(0, 3) = 3;

  (*cost_mat)(1, 0) = 0.9;
  (*cost_mat)(1, 1) = 2;
  (*cost_mat)(1, 2) = 3;
  (*cost_mat)(1, 3) = 8;

  (*cost_mat)(2, 0) = 4;
  (*cost_mat)(2, 1) = 3;
  (*cost_mat)(2, 2) = 0.3;
  (*cost_mat)(2, 3) = 0.1;

  //  matcher->set_max_match_distance(2.5);
  BipartiteGraphMatcherOptions options;
  options.cost_thresh = 2.5;
  options.bound_value = 100;
  std::vector<std::pair<size_t, size_t>> assignments;
  std::vector<size_t> unassigned_rows;
  std::vector<size_t> unassigned_cols;
  matcher->Match(options, &assignments, &unassigned_rows, &unassigned_cols);
  for (int i = 0; i < assignments.size() - 1; i++) {
    for (int j = i + 1; j < assignments.size(); j++) {
      if (assignments[i].first > assignments[j].first) {
        std::swap(assignments[i], assignments[j]);
      }
    }
  }

  CHECK_EQ(assignments.size(), 3);
  CHECK_EQ(unassigned_rows.size(), 0);
  CHECK_EQ(unassigned_cols.size(), 1);

  CHECK_EQ(assignments[0].first, 0);
  CHECK_EQ(assignments[0].second, 1);
  CHECK_EQ(assignments[1].first, 1);
  CHECK_EQ(assignments[1].second, 0);
  CHECK_EQ(assignments[2].first, 2);
  CHECK_EQ(assignments[2].second, 3);
  CHECK_EQ(unassigned_cols[0], 2);
  delete matcher;
}

TEST(BipartiteGraphMatcherTest, gnn_bipartite_graph_matcher_test) {
  GnnBipartiteGraphMatcher *matcher = new GnnBipartiteGraphMatcher();
  common::SecureMat<float> *cost_mat = matcher->cost_matrix();
  cost_mat->reserve(10, 10);
  cost_mat->resize(3, 4);
  (*cost_mat)(0, 0) = 0.2;
  (*cost_mat)(0, 1) = 1.2;
  (*cost_mat)(0, 2) = 4;
  (*cost_mat)(0, 3) = 3;

  (*cost_mat)(1, 0) = 0.9;
  (*cost_mat)(1, 1) = 2;
  (*cost_mat)(1, 2) = 3;
  (*cost_mat)(1, 3) = 8;

  (*cost_mat)(2, 0) = 4;
  (*cost_mat)(2, 1) = 3;
  (*cost_mat)(2, 2) = 0.3;
  (*cost_mat)(2, 3) = 0.1;
  // matcher->set_max_match_distance(2.5);
  BipartiteGraphMatcherOptions options;
  options.cost_thresh = 2.5;
  options.bound_value = 100;
  std::vector<std::pair<size_t, size_t>> assignments;
  std::vector<size_t> unassigned_rows;
  std::vector<size_t> unassigned_cols;
  matcher->Match(options, &assignments, &unassigned_rows, &unassigned_cols);
  for (int i = 0; i < assignments.size() - 1; i++) {
    for (int j = i + 1; j < assignments.size(); j++) {
      if (assignments[i].first > assignments[j].first) {
        std::swap(assignments[i], assignments[j]);
      }
    }
  }

  // results: (2, 3), (0, 0), (1, 1)
  CHECK_EQ(assignments.size(), 3);
  CHECK_EQ(unassigned_rows.size(), 0);
  CHECK_EQ(unassigned_cols.size(), 1);

  CHECK_EQ(assignments[0].first, 0);
  CHECK_EQ(assignments[0].second, 0);
  CHECK_EQ(assignments[1].first, 1);
  CHECK_EQ(assignments[1].second, 1);
  CHECK_EQ(assignments[2].first, 2);
  CHECK_EQ(assignments[2].second, 3);
  CHECK_EQ(unassigned_cols[0], 2);
  delete matcher;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
