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

#include "modules/perception/common/graph/gated_hungarian_bigraph_matcher.h"

#include "Eigen/Core"
#include "gtest/gtest.h"

namespace apollo {
namespace perception {
namespace common {

class GatedHungarianMatcherTest : public testing::Test {
 public:
  GatedHungarianMatcherTest() {}
  ~GatedHungarianMatcherTest() {}

 protected:
  void SetUp() { optimizer_ = new GatedHungarianMatcher<float>(1000); }
  void TearDown() {
    delete optimizer_;
    optimizer_ = nullptr;
  }

 public:
  GatedHungarianMatcher<float>* optimizer_ = nullptr;
};  // class HungarianMatcherTest

TEST_F(GatedHungarianMatcherTest, test_Match_Minimize_badcase1) {
  SecureMat<float>* global_costs = optimizer_->mutable_global_costs();
  global_costs->Reserve(1000, 1000);

  float bound_value = 10.0f;
  float cost_thresh = 2.5f;
  GatedHungarianMatcher<float>::OptimizeFlag opt_flag =
      GatedHungarianMatcher<float>::OptimizeFlag::OPTMIN;
  std::vector<std::pair<size_t, size_t>> assignments;
  std::vector<size_t> unassigned_rows;
  std::vector<size_t> unassigned_cols;
  global_costs->Resize(16, 16);
  for (size_t i = 0; i < 16; ++i) {
    for (size_t j = 0; j < 16; ++j) {
      (*global_costs)(i, j) = 10;
    }
  }
  (*global_costs)(1, 0) = 0.265f;
  (*global_costs)(2, 1) = 1.824f;
  (*global_costs)(3, 2) = 1.785f;
  (*global_costs)(3, 11) = 0.309f;
  (*global_costs)(4, 3) = 1.784f;
  (*global_costs)(5, 4) = 1.809f;
  (*global_costs)(6, 5) = 1.775f;
  (*global_costs)(7, 6) = 1.798f;
  (*global_costs)(8, 7) = 1.814f;
  (*global_costs)(9, 8) = 1.721f;
  (*global_costs)(10, 9) = 1.904f;
  (*global_costs)(11, 10) = 1.816f;
  (*global_costs)(12, 11) = 1.785f;
  (*global_costs)(13, 12) = 1.954f;
  (*global_costs)(14, 13) = 0.247f;
  (*global_costs)(15, 15) = 1.814f;

  optimizer_->Match(cost_thresh, bound_value, opt_flag, &assignments,
                    &unassigned_rows, &unassigned_cols);
  EXPECT_EQ(15, assignments.size());
  EXPECT_EQ(1, assignments[0].first);
  EXPECT_EQ(0, assignments[0].second);
  EXPECT_EQ(2, assignments[1].first);
  EXPECT_EQ(1, assignments[1].second);
  EXPECT_EQ(3, assignments[2].first);
  EXPECT_EQ(2, assignments[2].second);
  EXPECT_EQ(12, assignments[3].first);
  EXPECT_EQ(11, assignments[3].second);
  EXPECT_EQ(4, assignments[4].first);
  EXPECT_EQ(3, assignments[4].second);
  EXPECT_EQ(5, assignments[5].first);
  EXPECT_EQ(4, assignments[5].second);
  EXPECT_EQ(6, assignments[6].first);
  EXPECT_EQ(5, assignments[6].second);
  EXPECT_EQ(7, assignments[7].first);
  EXPECT_EQ(6, assignments[7].second);
  EXPECT_EQ(8, assignments[8].first);
  EXPECT_EQ(7, assignments[8].second);
  EXPECT_EQ(9, assignments[9].first);
  EXPECT_EQ(8, assignments[9].second);
  EXPECT_EQ(10, assignments[10].first);
  EXPECT_EQ(9, assignments[10].second);
  EXPECT_EQ(11, assignments[11].first);
  EXPECT_EQ(10, assignments[11].second);
  EXPECT_EQ(13, assignments[12].first);
  EXPECT_EQ(12, assignments[12].second);
  EXPECT_EQ(14, assignments[13].first);
  EXPECT_EQ(13, assignments[13].second);
  EXPECT_EQ(15, assignments[14].first);
  EXPECT_EQ(15, assignments[14].second);
}

TEST_F(GatedHungarianMatcherTest, test_Match_Minimize_badcase_2) {
  SecureMat<float>* global_costs = optimizer_->mutable_global_costs();
  global_costs->Reserve(1000, 1000);

  float bound_value = 10.0f;
  float cost_thresh = 2.5f;
  GatedHungarianMatcher<float>::OptimizeFlag opt_flag =
      GatedHungarianMatcher<float>::OptimizeFlag::OPTMIN;
  std::vector<std::pair<size_t, size_t>> assignments;
  std::vector<size_t> unassigned_rows;
  std::vector<size_t> unassigned_cols;
  global_costs->Resize(9, 9);
  for (size_t i = 0; i < 9; ++i) {
    for (size_t j = 0; j < 9; ++j) {
      (*global_costs)(i, j) = 10.0f;
    }
  }
  (*global_costs)(0, 0) = 1.834f;
  (*global_costs)(1, 2) = 1.826f;
  (*global_costs)(2, 1) = 1.854f;
  (*global_costs)(3, 3) = 1.930f;
  (*global_costs)(4, 4) = 1.965f;
  (*global_costs)(5, 0) = 0.677f;
  (*global_costs)(5, 5) = 1.835f;
  (*global_costs)(6, 6) = 1.553f;
  (*global_costs)(7, 7) = 1.844f;
  (*global_costs)(8, 8) = 1.844f;

  optimizer_->Match(cost_thresh, bound_value, opt_flag, &assignments,
                    &unassigned_rows, &unassigned_cols);
  EXPECT_EQ(9, assignments.size());
  EXPECT_EQ(0, assignments[0].first);
  EXPECT_EQ(0, assignments[0].second);
  EXPECT_EQ(5, assignments[1].first);
  EXPECT_EQ(5, assignments[1].second);
  EXPECT_EQ(1, assignments[2].first);
  EXPECT_EQ(2, assignments[2].second);
  EXPECT_EQ(2, assignments[3].first);
  EXPECT_EQ(1, assignments[3].second);
  EXPECT_EQ(3, assignments[4].first);
  EXPECT_EQ(3, assignments[4].second);
  EXPECT_EQ(4, assignments[5].first);
  EXPECT_EQ(4, assignments[5].second);
  EXPECT_EQ(6, assignments[6].first);
  EXPECT_EQ(6, assignments[6].second);
  EXPECT_EQ(7, assignments[7].first);
  EXPECT_EQ(7, assignments[7].second);
  EXPECT_EQ(8, assignments[8].first);
  EXPECT_EQ(8, assignments[8].second);
}

TEST_F(GatedHungarianMatcherTest, test_Match_Minimize) {
  SecureMat<float>* global_costs = optimizer_->mutable_global_costs();
  global_costs->Reserve(1000, 1000);

  float cost_thresh = 1.0f;
  float bound_value = 2.0f;
  GatedHungarianMatcher<float>::OptimizeFlag opt_flag =
      GatedHungarianMatcher<float>::OptimizeFlag::OPTMIN;
  std::vector<std::pair<size_t, size_t>> assignments;
  std::vector<size_t> unassigned_rows;
  std::vector<size_t> unassigned_cols;

  /* case 1: most basic one
   * costs:
   * 0.1,  2.0
   * 2.0,  0.1
   * matches:
   * (0->1, 1->0) */
  cost_thresh = 1.0;
  global_costs->Resize(2, 2);
  (*global_costs)(0, 0) = 0.1f;
  (*global_costs)(1, 0) = 2.0f;
  (*global_costs)(0, 1) = 2.0f;
  (*global_costs)(1, 1) = 0.1f;

  optimizer_->Match(cost_thresh, opt_flag, &assignments, &unassigned_rows,
                    &unassigned_cols);
  EXPECT_EQ(2, assignments.size());
  EXPECT_EQ(0, assignments[0].first);
  EXPECT_EQ(0, assignments[0].second);
  EXPECT_EQ(1, assignments[1].first);
  EXPECT_EQ(1, assignments[1].second);
  EXPECT_EQ(0, unassigned_rows.size());
  EXPECT_EQ(0, unassigned_cols.size());

  /* case 2: special one with all 0s
   * costs:
   * 0.0,  0.0
   * 0.0,  0.0
   * matches:
   * (0->0, 1->1) */
  cost_thresh = 1.0;
  global_costs->Resize(2, 2);
  (*global_costs)(0, 0) = 0.0f;
  (*global_costs)(0, 1) = 0.0f;
  (*global_costs)(1, 0) = 0.0f;
  (*global_costs)(1, 1) = 0.0f;

  optimizer_->Match(cost_thresh, opt_flag, &assignments, &unassigned_rows,
                    &unassigned_cols);
  EXPECT_EQ(2, assignments.size());
  EXPECT_EQ(0, assignments[0].first);
  EXPECT_EQ(0, assignments[0].second);
  EXPECT_EQ(1, assignments[1].first);
  EXPECT_EQ(1, assignments[1].second);
  EXPECT_EQ(0, unassigned_rows.size());
  EXPECT_EQ(0, unassigned_cols.size());

  /* case 3: special one with all the same values
   * costs:
   * 3.0,  3.0
   * 3.0,  3.0
   * matches:
   * (0->0, 1->1) */
  cost_thresh = 1.0;
  global_costs->Resize(2, 2);
  (*global_costs)(0, 0) = 3.0f;
  (*global_costs)(0, 1) = 3.0f;
  (*global_costs)(1, 0) = 3.0f;
  (*global_costs)(1, 1) = 3.0f;

  optimizer_->Match(cost_thresh, opt_flag, &assignments, &unassigned_rows,
                    &unassigned_cols);
  EXPECT_EQ(0, assignments.size());
  EXPECT_EQ(2, unassigned_rows.size());
  EXPECT_EQ(0, unassigned_rows[0]);
  EXPECT_EQ(1, unassigned_rows[1]);
  EXPECT_EQ(2, unassigned_cols.size());
  EXPECT_EQ(0, unassigned_cols[0]);
  EXPECT_EQ(1, unassigned_cols[1]);

  /* case 4: further one with more cols
   * costs:
   * 4.7,  3.8,  1.0,  2.0
   * 4.1,  3.0,  2.0,  3.0
   * 1.0,  2.0,  4.7,  4.9
   * if cost_thresh = 1.0, matches: empty
   * if cost_thresh = 2.0, matches: (0->2, 2->0)
   * if cost_thresh = 4.0, matches: (0->2, 1->1, 2->0) */
  cost_thresh = 1.0;
  global_costs->Resize(3, 4);
  (*global_costs)(0, 0) = 4.7f;
  (*global_costs)(0, 1) = 3.8f;
  (*global_costs)(0, 2) = 1.0f;
  (*global_costs)(0, 3) = 2.0f;
  (*global_costs)(1, 0) = 4.1f;
  (*global_costs)(1, 1) = 3.0f;
  (*global_costs)(1, 2) = 2.0f;
  (*global_costs)(1, 3) = 3.0f;
  (*global_costs)(2, 0) = 1.0f;
  (*global_costs)(2, 1) = 2.0f;
  (*global_costs)(2, 2) = 4.7f;
  (*global_costs)(2, 3) = 4.9f;

  optimizer_->Match(cost_thresh, opt_flag, &assignments, &unassigned_rows,
                    &unassigned_cols);
  EXPECT_EQ(0, assignments.size());
  EXPECT_EQ(3, unassigned_rows.size());
  EXPECT_EQ(0, unassigned_rows[0]);
  EXPECT_EQ(1, unassigned_rows[1]);
  EXPECT_EQ(2, unassigned_rows[2]);
  EXPECT_EQ(4, unassigned_cols.size());
  EXPECT_EQ(0, unassigned_cols[0]);
  EXPECT_EQ(1, unassigned_cols[1]);
  EXPECT_EQ(2, unassigned_cols[2]);
  EXPECT_EQ(3, unassigned_cols[3]);

  cost_thresh = 2.0;
  optimizer_->Match(cost_thresh, opt_flag, &assignments, &unassigned_rows,
                    &unassigned_cols);
  EXPECT_EQ(2, assignments.size());
  EXPECT_EQ(0, assignments[0].first);
  EXPECT_EQ(2, assignments[0].second);
  EXPECT_EQ(2, assignments[1].first);
  EXPECT_EQ(0, assignments[1].second);
  EXPECT_EQ(1, unassigned_rows.size());
  EXPECT_EQ(1, unassigned_rows[0]);
  EXPECT_EQ(2, unassigned_cols.size());
  EXPECT_EQ(1, unassigned_cols[0]);
  EXPECT_EQ(3, unassigned_cols[1]);

  cost_thresh = 4.0;
  optimizer_->Match(cost_thresh, opt_flag, &assignments, &unassigned_rows,
                    &unassigned_cols);
  EXPECT_EQ(3, assignments.size());
  EXPECT_EQ(0, assignments[0].first);
  EXPECT_EQ(2, assignments[0].second);
  EXPECT_EQ(1, assignments[1].first);
  EXPECT_EQ(1, assignments[1].second);
  EXPECT_EQ(2, assignments[2].first);
  EXPECT_EQ(0, assignments[2].second);
  EXPECT_EQ(0, unassigned_rows.size());
  EXPECT_EQ(1, unassigned_cols.size());
  EXPECT_EQ(3, unassigned_cols[1]);

  /* case 5: further one with more rows
   * costs:
   * 4.7,  4.9,  1.0,  4.2,  1.5,  4.3
   * 4.1,  5.0,  2.0,  5.3,  2.3,  4.3
   * 1.0,  2.0,  4.7,  4.9,  4.5,  4.5
   * 3.2,  2.1,  4.5,  0.4,  4.9,  4.9
   * 1.0,  2.1,  2.1,  2.1,  3.0,  4.0
   * 3.0,  3.3,  2.1,  2.0,  3.9,  4.9
   * 3.2,  1.1,  4.1,  3.7,  4.9,  4.9
   * if cost_thresh = 4.0, matches: (0->2, 1->4, 2->0, 3->3, 6->1)
   * if cost_thresh = 4.0 & bound_value = 10, matches:
   * (0->2, 1->4, 2->0, 3->3, 6->1) */
  cost_thresh = 4.0;
  global_costs->Resize(7, 6);
  (*global_costs)(0, 0) = 4.7f;
  (*global_costs)(0, 1) = 4.9f;
  (*global_costs)(0, 2) = 1.0f;
  (*global_costs)(0, 3) = 4.2f;
  (*global_costs)(0, 4) = 1.5f;
  (*global_costs)(0, 5) = 4.3f;
  (*global_costs)(1, 0) = 4.1f;
  (*global_costs)(1, 1) = 5.0f;
  (*global_costs)(1, 2) = 2.0f;
  (*global_costs)(1, 3) = 5.3f;
  (*global_costs)(1, 4) = 2.3f;
  (*global_costs)(1, 5) = 4.3f;
  (*global_costs)(2, 0) = 1.0f;
  (*global_costs)(2, 1) = 2.0f;
  (*global_costs)(2, 2) = 4.7f;
  (*global_costs)(2, 3) = 4.9f;
  (*global_costs)(2, 4) = 4.5f;
  (*global_costs)(2, 5) = 4.5f;
  (*global_costs)(3, 0) = 3.2f;
  (*global_costs)(3, 1) = 2.1f;
  (*global_costs)(3, 2) = 4.5f;
  (*global_costs)(3, 3) = 0.4f;
  (*global_costs)(3, 4) = 4.9f;
  (*global_costs)(3, 5) = 4.9f;
  (*global_costs)(4, 0) = 1.0f;
  (*global_costs)(4, 1) = 2.1f;
  (*global_costs)(4, 2) = 2.1f;
  (*global_costs)(4, 3) = 2.1f;
  (*global_costs)(4, 4) = 3.0f;
  (*global_costs)(4, 5) = 4.0f;
  (*global_costs)(5, 0) = 3.0f;
  (*global_costs)(5, 1) = 3.3f;
  (*global_costs)(5, 2) = 2.1f;
  (*global_costs)(5, 3) = 2.0f;
  (*global_costs)(5, 4) = 3.9f;
  (*global_costs)(5, 5) = 4.9f;
  (*global_costs)(6, 0) = 3.2f;
  (*global_costs)(6, 1) = 1.1f;
  (*global_costs)(6, 2) = 4.1f;
  (*global_costs)(6, 3) = 3.7f;
  (*global_costs)(6, 4) = 4.9f;
  (*global_costs)(6, 5) = 4.9f;

  optimizer_->Match(cost_thresh, opt_flag, &assignments, &unassigned_rows,
                    &unassigned_cols);
  EXPECT_EQ(5, assignments.size());
  EXPECT_EQ(0, assignments[0].first);
  EXPECT_EQ(2, assignments[0].second);
  EXPECT_EQ(1, assignments[1].first);
  EXPECT_EQ(4, assignments[1].second);
  EXPECT_EQ(2, assignments[2].first);
  EXPECT_EQ(0, assignments[2].second);
  EXPECT_EQ(3, assignments[3].first);
  EXPECT_EQ(3, assignments[3].second);
  EXPECT_EQ(6, assignments[4].first);
  EXPECT_EQ(1, assignments[4].second);
  EXPECT_EQ(2, unassigned_rows.size());
  EXPECT_EQ(4, unassigned_rows[0]);
  EXPECT_EQ(5, unassigned_rows[1]);
  EXPECT_EQ(1, unassigned_cols.size());
  EXPECT_EQ(5, unassigned_cols[0]);

  cost_thresh = 4.0;
  bound_value = 10;
  optimizer_->Match(cost_thresh, bound_value, opt_flag, &assignments,
                    &unassigned_rows, &unassigned_cols);
  EXPECT_EQ(5, assignments.size());
  EXPECT_EQ(0, assignments[0].first);
  EXPECT_EQ(2, assignments[0].second);
  EXPECT_EQ(1, assignments[1].first);
  EXPECT_EQ(4, assignments[1].second);
  EXPECT_EQ(2, assignments[2].first);
  EXPECT_EQ(0, assignments[2].second);
  EXPECT_EQ(3, assignments[3].first);
  EXPECT_EQ(3, assignments[3].second);
  EXPECT_EQ(6, assignments[4].first);
  EXPECT_EQ(1, assignments[4].second);
  EXPECT_EQ(2, unassigned_rows.size());
  EXPECT_EQ(4, unassigned_rows[0]);
  EXPECT_EQ(5, unassigned_rows[1]);
  EXPECT_EQ(1, unassigned_cols.size());
  EXPECT_EQ(5, unassigned_cols[0]);

  /* case 6: empty one */
  global_costs->Resize(0, 0);
  optimizer_->Match(cost_thresh, opt_flag, &assignments, &unassigned_rows,
                    &unassigned_cols);
  EXPECT_EQ(0, assignments.size());
  EXPECT_EQ(0, unassigned_rows.size());
  EXPECT_EQ(0, unassigned_rows.size());
}

TEST_F(GatedHungarianMatcherTest, test_Match_Maximize) {
  SecureMat<float>* global_costs = optimizer_->mutable_global_costs();
  global_costs->Reserve(1000, 1000);

  float cost_thresh = 1.0f;
  float bound_value = 2.0f;
  GatedHungarianMatcher<float>::OptimizeFlag opt_flag =
      GatedHungarianMatcher<float>::OptimizeFlag::OPTMAX;
  std::vector<std::pair<size_t, size_t>> assignments;
  std::vector<size_t> unassigned_rows;
  std::vector<size_t> unassigned_cols;

  /* case 1: most basic one
   * costs:
   * 0.1,  2.0
   * 2.0,  0.1
   * matches:
   * (0->1, 1->0) */
  cost_thresh = 1.0;
  global_costs->Resize(2, 2);
  (*global_costs)(0, 0) = 0.1f;
  (*global_costs)(1, 0) = 2.0f;
  (*global_costs)(0, 1) = 2.0f;
  (*global_costs)(1, 1) = 0.1f;

  optimizer_->Match(cost_thresh, opt_flag, &assignments, &unassigned_rows,
                    &unassigned_cols);
  EXPECT_EQ(2, assignments.size());
  EXPECT_EQ(0, assignments[0].first);
  EXPECT_EQ(1, assignments[0].second);
  EXPECT_EQ(1, assignments[1].first);
  EXPECT_EQ(0, assignments[1].second);
  EXPECT_EQ(0, unassigned_rows.size());
  EXPECT_EQ(0, unassigned_cols.size());

  /* case 2: further one with more rows
   * costs:
   * 4.7,  4.9,  1.0,  4.2,  1.5,  4.3
   * 4.1,  5.0,  2.0,  5.3,  2.3,  4.3
   * 1.0,  2.0,  4.7,  4.9,  4.5,  4.5
   * 3.2,  2.1,  4.5,  0.4,  4.9,  4.9
   * 1.0,  2.1,  2.1,  2.1,  3.0,  4.0
   * 3.0,  3.3,  2.1,  2.0,  3.9,  4.9
   * 3.2,  1.1,  4.1,  3.7,  4.9,  4.9
   * if cost_thresh = 4.0, matches: (0->0, 1->1, 2->3, 3->2, 6->4)
   * if cost_thresho = 4.0 & bound_value = 0,
   * matches: (0->0, 1->1, 2->3, 3->2, 6->4) */
  cost_thresh = 4.0;
  global_costs->Resize(7, 6);
  (*global_costs)(0, 0) = 4.7f;
  (*global_costs)(0, 1) = 4.9f;
  (*global_costs)(0, 2) = 1.0f;
  (*global_costs)(0, 3) = 4.2f;
  (*global_costs)(0, 4) = 1.5f;
  (*global_costs)(0, 5) = 1.3f;
  (*global_costs)(1, 0) = 4.1f;
  (*global_costs)(1, 1) = 5.0f;
  (*global_costs)(1, 2) = 2.0f;
  (*global_costs)(1, 3) = 5.3f;
  (*global_costs)(1, 4) = 2.3f;
  (*global_costs)(1, 5) = 1.3f;
  (*global_costs)(2, 0) = 1.0f;
  (*global_costs)(2, 1) = 2.0f;
  (*global_costs)(2, 2) = 4.7f;
  (*global_costs)(2, 3) = 4.9f;
  (*global_costs)(2, 4) = 4.5f;
  (*global_costs)(2, 5) = 1.5f;
  (*global_costs)(3, 0) = 3.2f;
  (*global_costs)(3, 1) = 2.1f;
  (*global_costs)(3, 2) = 4.5f;
  (*global_costs)(3, 3) = 0.4f;
  (*global_costs)(3, 4) = 4.9f;
  (*global_costs)(3, 5) = 1.9f;
  (*global_costs)(4, 0) = 1.0f;
  (*global_costs)(4, 1) = 2.1f;
  (*global_costs)(4, 2) = 2.1f;
  (*global_costs)(4, 3) = 2.1f;
  (*global_costs)(4, 4) = 3.0f;
  (*global_costs)(4, 5) = 1.0f;
  (*global_costs)(5, 0) = 3.0f;
  (*global_costs)(5, 1) = 3.3f;
  (*global_costs)(5, 2) = 2.1f;
  (*global_costs)(5, 3) = 2.0f;
  (*global_costs)(5, 4) = 3.9f;
  (*global_costs)(5, 5) = 1.9f;
  (*global_costs)(6, 0) = 3.2f;
  (*global_costs)(6, 1) = 1.1f;
  (*global_costs)(6, 2) = 4.1f;
  (*global_costs)(6, 3) = 3.7f;
  (*global_costs)(6, 4) = 4.9f;
  (*global_costs)(6, 5) = 1.9f;

  optimizer_->Match(cost_thresh, opt_flag, &assignments, &unassigned_rows,
                    &unassigned_cols);
  EXPECT_EQ(5, assignments.size());
  EXPECT_EQ(0, assignments[0].first);
  EXPECT_EQ(0, assignments[0].second);
  EXPECT_EQ(1, assignments[1].first);
  EXPECT_EQ(1, assignments[1].second);
  EXPECT_EQ(2, assignments[2].first);
  EXPECT_EQ(3, assignments[2].second);
  EXPECT_EQ(3, assignments[3].first);
  EXPECT_EQ(2, assignments[3].second);
  EXPECT_EQ(6, assignments[4].first);
  EXPECT_EQ(4, assignments[4].second);
  EXPECT_EQ(2, unassigned_rows.size());
  EXPECT_EQ(4, unassigned_rows[0]);
  EXPECT_EQ(5, unassigned_rows[1]);
  EXPECT_EQ(1, unassigned_cols.size());
  EXPECT_EQ(5, unassigned_cols[0]);

  cost_thresh = 4.0;
  bound_value = 0;
  optimizer_->Match(cost_thresh, bound_value, opt_flag, &assignments,
                    &unassigned_rows, &unassigned_cols);
  EXPECT_EQ(5, assignments.size());
  EXPECT_EQ(0, assignments[0].first);
  EXPECT_EQ(0, assignments[0].second);
  EXPECT_EQ(1, assignments[1].first);
  EXPECT_EQ(1, assignments[1].second);
  EXPECT_EQ(2, assignments[2].first);
  EXPECT_EQ(3, assignments[2].second);
  EXPECT_EQ(3, assignments[3].first);
  EXPECT_EQ(2, assignments[3].second);
  EXPECT_EQ(6, assignments[4].first);
  EXPECT_EQ(4, assignments[4].second);
  EXPECT_EQ(2, unassigned_rows.size());
  EXPECT_EQ(4, unassigned_rows[0]);
  EXPECT_EQ(5, unassigned_rows[1]);
  EXPECT_EQ(1, unassigned_cols.size());
  EXPECT_EQ(5, unassigned_cols[0]);

  /* case 3: empty one */
  global_costs->Resize(0, 0);
  optimizer_->Match(cost_thresh, opt_flag, &assignments, &unassigned_rows,
                    &unassigned_cols);
  EXPECT_EQ(0, assignments.size());
  EXPECT_EQ(0, unassigned_rows.size());
  EXPECT_EQ(0, unassigned_rows.size());
}

}  // namespace common
}  // namespace perception
}  // namespace apollo
