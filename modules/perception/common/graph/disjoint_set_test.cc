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

#include "modules/perception/common/graph/disjoint_set.h"

#include <memory>

#include "gtest/gtest.h"

namespace apollo {
namespace perception {
namespace common {

class DisjointSetTest : public testing::Test {
 protected:
  void SetUp() { universe_ = std::shared_ptr<Universe>(new Universe(16)); }

  std::shared_ptr<Universe> universe_;
};

TEST_F(DisjointSetTest, test_get_sets_num) {
  EXPECT_EQ(16, universe_->GetSetsNum());
}

TEST_F(DisjointSetTest, test_get_size) {
  EXPECT_EQ(1, universe_->GetSize(0));
  EXPECT_EQ(1, universe_->GetSize(5));
  EXPECT_EQ(1, universe_->GetSize(12));
}

TEST_F(DisjointSetTest, test_reset) {
  universe_->Reset(10);
  EXPECT_EQ(10, universe_->GetSetsNum());
  EXPECT_EQ(1, universe_->GetSize(7));
}

TEST_F(DisjointSetTest, test_find_join) {
  EXPECT_EQ(6, universe_->Find(6));
  EXPECT_EQ(9, universe_->Find(9));
  universe_->Join(6, 9);
  EXPECT_EQ(9, universe_->Find(6));
  EXPECT_EQ(2, universe_->GetSize(9));
  EXPECT_EQ(15, universe_->GetSetsNum());
  universe_->Join(9, 1);
  EXPECT_EQ(9, universe_->Find(1));
  EXPECT_EQ(3, universe_->GetSize(9));
  EXPECT_EQ(14, universe_->GetSetsNum());
  universe_->Join(2, 4);
  universe_->Join(4, 9);
  EXPECT_EQ(9, universe_->Find(4));
  EXPECT_EQ(5, universe_->GetSize(9));
  EXPECT_EQ(12, universe_->GetSetsNum());
}

}  // namespace common
}  // namespace perception
}  // namespace apollo
