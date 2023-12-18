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

#include "modules/perception/common/algorithm/graph/hungarian_optimizer.h"

#include "Eigen/Core"
#include "gtest/gtest.h"

namespace apollo {
namespace perception {
namespace algorithm {

class HungarianOptimizerTest : public testing::Test {
 public:
  HungarianOptimizerTest() : optimizer_(nullptr) {}
  ~HungarianOptimizerTest() {}

 protected:
  void SetUp() { optimizer_ = new HungarianOptimizer<float>(); }
  void TearDown() {
    delete optimizer_;
    optimizer_ = nullptr;
  }

 public:
  HungarianOptimizer<float>* optimizer_;
};  // class HungarianOptimizerTest

TEST_F(HungarianOptimizerTest, test_Minimize) {
  std::vector<std::pair<size_t, size_t>> assignments;
  optimizer_->costs()->Reserve(1000, 1000);

  /* case 1: most basic one
   * costs:
   * 0.1,  1.0
   * 1.0,  0.1
   * matches:
   * (0->0, 1->1) */
  optimizer_->costs()->Resize(2, 2);
  (*optimizer_->costs())(0, 0) = 0.1f;
  (*optimizer_->costs())(1, 0) = 1.0f;
  (*optimizer_->costs())(0, 1) = 1.0f;
  (*optimizer_->costs())(1, 1) = 0.1f;

  optimizer_->Minimize(&assignments);
  optimizer_->PrintMatrix();

  EXPECT_EQ(2, assignments.size());
  EXPECT_EQ(0, assignments[0].first);
  EXPECT_EQ(0, assignments[0].second);
  EXPECT_EQ(1, assignments[1].first);
  EXPECT_EQ(1, assignments[1].second);

  /* case 2: special one with all 0s
   * costs:
   * 0.0,  0.0
   * 0.0,  0.0
   * matches:
   * (0->0, 1->1) */
  optimizer_->costs()->Resize(2, 2);
  (*optimizer_->costs())(0, 0) = 0.0f;
  (*optimizer_->costs())(0, 1) = 0.0f;
  (*optimizer_->costs())(1, 0) = 0.0f;
  (*optimizer_->costs())(1, 1) = 0.0f;

  optimizer_->Minimize(&assignments);

  EXPECT_EQ(2, assignments.size());
  EXPECT_EQ(0, assignments[0].first);
  EXPECT_EQ(0, assignments[0].second);
  EXPECT_EQ(1, assignments[1].first);
  EXPECT_EQ(1, assignments[1].second);

  /* case 3: special one with all the same values
   * costs:
   * 3.0,  3.0
   * 3.0,  3.0
   * matches:
   * (0->0, 1->1) */
  optimizer_->costs()->Resize(2, 2);
  (*optimizer_->costs())(0, 0) = 3.0f;
  (*optimizer_->costs())(0, 1) = 3.0f;
  (*optimizer_->costs())(1, 0) = 3.0f;
  (*optimizer_->costs())(1, 1) = 3.0f;

  optimizer_->Minimize(&assignments);

  EXPECT_EQ(2, assignments.size());
  EXPECT_EQ(0, assignments[0].first);
  EXPECT_EQ(0, assignments[0].second);
  EXPECT_EQ(1, assignments[1].first);
  EXPECT_EQ(1, assignments[1].second);

  /* case 4: further one with more cols
   * costs:
   * 4.7,  3.8,  1.0,  2.0
   * 4.1,  3.0,  2.0,  3.0
   * 1.0,  2.0,  4.7,  4.9
   * matches:
   * (0->2, 1->1, 2->0) */
  optimizer_->costs()->Resize(3, 4);
  (*optimizer_->costs())(0, 0) = 4.7f;
  (*optimizer_->costs())(0, 1) = 3.8f;
  (*optimizer_->costs())(0, 2) = 1.0f;
  (*optimizer_->costs())(0, 3) = 2.0f;
  (*optimizer_->costs())(1, 0) = 4.1f;
  (*optimizer_->costs())(1, 1) = 3.0f;
  (*optimizer_->costs())(1, 2) = 2.0f;
  (*optimizer_->costs())(1, 3) = 3.0f;
  (*optimizer_->costs())(2, 0) = 1.0f;
  (*optimizer_->costs())(2, 1) = 2.0f;
  (*optimizer_->costs())(2, 2) = 4.7f;
  (*optimizer_->costs())(2, 3) = 4.9f;

  optimizer_->Minimize(&assignments);

  EXPECT_EQ(3, assignments.size());
  EXPECT_EQ(0, assignments[0].first);
  EXPECT_EQ(2, assignments[0].second);
  EXPECT_EQ(1, assignments[1].first);
  EXPECT_EQ(1, assignments[1].second);
  EXPECT_EQ(2, assignments[2].first);
  EXPECT_EQ(0, assignments[2].second);

  /* case 5: further one with more rows
   * costs:
   * 4.7,  3.8,  1.0
   * 4.1,  3.0,  2.0
   * 1.0,  2.0,  4.7
   * 3.2,  2.1,  0.5
   * matches:
   * (0->2, 2->0, 3->1) */
  optimizer_->costs()->Resize(4, 3);
  (*optimizer_->costs())(0, 0) = 4.7f;
  (*optimizer_->costs())(0, 1) = 3.8f;
  (*optimizer_->costs())(0, 2) = 1.0f;
  (*optimizer_->costs())(1, 0) = 4.1f;
  (*optimizer_->costs())(1, 1) = 3.0f;
  (*optimizer_->costs())(1, 2) = 2.0f;
  (*optimizer_->costs())(2, 0) = 1.0f;
  (*optimizer_->costs())(2, 1) = 2.0f;
  (*optimizer_->costs())(2, 2) = 4.7f;
  (*optimizer_->costs())(3, 0) = 3.2f;
  (*optimizer_->costs())(3, 1) = 2.1f;
  (*optimizer_->costs())(3, 2) = 0.5f;

  optimizer_->Minimize(&assignments);

  EXPECT_EQ(3, assignments.size());
  EXPECT_EQ(0, assignments[0].first);
  EXPECT_EQ(2, assignments[0].second);
  EXPECT_EQ(2, assignments[1].first);
  EXPECT_EQ(0, assignments[1].second);
  EXPECT_EQ(3, assignments[2].first);
  EXPECT_EQ(1, assignments[2].second);

  /* case 6: empty one */
  optimizer_->costs()->Resize(0, 0);
  optimizer_->Minimize(&assignments);
  EXPECT_EQ(0, assignments.size());
}

TEST_F(HungarianOptimizerTest, test_Maximize) {
  std::vector<std::pair<size_t, size_t>> assignments;
  optimizer_->costs()->Reserve(1000, 1000);

  /* case 1: most basic one
   * costs:
   * 0.1,  1.0
   * 1.0,  0.1
   * matches:
   * (0->1, 1->0) */
  optimizer_->costs()->Resize(2, 2);
  (*optimizer_->costs())(0, 0) = 0.1f;
  (*optimizer_->costs())(1, 0) = 1.0f;
  (*optimizer_->costs())(0, 1) = 1.0f;
  (*optimizer_->costs())(1, 1) = 0.1f;

  optimizer_->Maximize(&assignments);

  EXPECT_EQ(2, assignments.size());
  EXPECT_EQ(0, assignments[0].first);
  EXPECT_EQ(1, assignments[0].second);
  EXPECT_EQ(1, assignments[1].first);
  EXPECT_EQ(0, assignments[1].second);

  /* case 2: special one with all 0s
   * costs:
   * 0.0,  0.0
   * 0.0,  0.0
   * matches:
   * (0->0, 1->1) */
  optimizer_->costs()->Resize(2, 2);
  (*optimizer_->costs())(0, 0) = 0.0f;
  (*optimizer_->costs())(0, 1) = 0.0f;
  (*optimizer_->costs())(1, 0) = 0.0f;
  (*optimizer_->costs())(1, 1) = 0.0f;

  optimizer_->Maximize(&assignments);

  EXPECT_EQ(2, assignments.size());
  EXPECT_EQ(0, assignments[0].first);
  EXPECT_EQ(0, assignments[0].second);
  EXPECT_EQ(1, assignments[1].first);
  EXPECT_EQ(1, assignments[1].second);

  /* case 3: special one with all the same values
   * costs:
   * 3.0,  3.0
   * 3.0,  3.0
   * matches:
   * (0->0, 1->1) */
  optimizer_->costs()->Resize(2, 2);
  (*optimizer_->costs())(0, 0) = 3.0f;
  (*optimizer_->costs())(0, 1) = 3.0f;
  (*optimizer_->costs())(1, 0) = 3.0f;
  (*optimizer_->costs())(1, 1) = 3.0f;

  optimizer_->Maximize(&assignments);

  EXPECT_EQ(2, assignments.size());
  EXPECT_EQ(0, assignments[0].first);
  EXPECT_EQ(0, assignments[0].second);
  EXPECT_EQ(1, assignments[1].first);
  EXPECT_EQ(1, assignments[1].second);

  /* case 4: further one with more cols
   * costs:
   * 4.7,  3.8,  1.0,  2.0
   * 4.1,  3.0,  2.0,  3.0
   * 1.0,  2.0,  4.7,  4.9
   * matches:
   * (0->1, 1->0, 2->3) */
  optimizer_->costs()->Resize(3, 4);
  (*optimizer_->costs())(0, 0) = 4.7f;
  (*optimizer_->costs())(0, 1) = 3.8f;
  (*optimizer_->costs())(0, 2) = 1.0f;
  (*optimizer_->costs())(0, 3) = 2.0f;
  (*optimizer_->costs())(1, 0) = 4.1f;
  (*optimizer_->costs())(1, 1) = 3.0f;
  (*optimizer_->costs())(1, 2) = 2.0f;
  (*optimizer_->costs())(1, 3) = 3.0f;
  (*optimizer_->costs())(2, 0) = 1.0f;
  (*optimizer_->costs())(2, 1) = 2.0f;
  (*optimizer_->costs())(2, 2) = 4.7f;
  (*optimizer_->costs())(2, 3) = 4.9f;

  optimizer_->Maximize(&assignments);

  EXPECT_EQ(3, assignments.size());
  EXPECT_EQ(0, assignments[0].first);
  EXPECT_EQ(1, assignments[0].second);
  EXPECT_EQ(1, assignments[1].first);
  EXPECT_EQ(0, assignments[1].second);
  EXPECT_EQ(2, assignments[2].first);
  EXPECT_EQ(3, assignments[2].second);

  /* case 5: further one with more rows
   * costs:
   * 4.7,  3.8,  1.0
   * 4.1,  3.0,  2.0
   * 1.0,  2.0,  4.7
   * 3.2,  2.1,  0.5
   * matches:
   * (0->1, 1->0, 2->2) */
  optimizer_->costs()->Resize(4, 3);
  (*optimizer_->costs())(0, 0) = 4.7f;
  (*optimizer_->costs())(0, 1) = 3.8f;
  (*optimizer_->costs())(0, 2) = 1.0f;
  (*optimizer_->costs())(1, 0) = 4.1f;
  (*optimizer_->costs())(1, 1) = 3.0f;
  (*optimizer_->costs())(1, 2) = 2.0f;
  (*optimizer_->costs())(2, 0) = 1.0f;
  (*optimizer_->costs())(2, 1) = 2.0f;
  (*optimizer_->costs())(2, 2) = 4.7f;
  (*optimizer_->costs())(3, 0) = 3.2f;
  (*optimizer_->costs())(3, 1) = 2.1f;
  (*optimizer_->costs())(3, 2) = 0.5f;

  optimizer_->Maximize(&assignments);

  EXPECT_EQ(3, assignments.size());
  EXPECT_EQ(0, assignments[0].first);
  EXPECT_EQ(1, assignments[0].second);
  EXPECT_EQ(1, assignments[1].first);
  EXPECT_EQ(0, assignments[1].second);
  EXPECT_EQ(2, assignments[2].first);
  EXPECT_EQ(2, assignments[2].second);

  /* case 6: empty one */
  optimizer_->costs()->Resize(0, 0);
  optimizer_->Maximize(&assignments);
  EXPECT_EQ(0, assignments.size());
}

}  // namespace algorithm
}  // namespace perception
}  // namespace apollo
