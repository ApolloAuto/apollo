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

/**
 * @file
 * @brief This file provides several unit tests for the class
 * "TrajectorySmoother".
 */
#include "modules/tools/navi_generator/backend/util/trajectory_smoother.h"

#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "modules/common/util/util.h"

namespace apollo {
namespace navi_generator {
namespace util {

class TrajectorySmootherTest : public testing::Test {
 public:
  virtual void SetUp() {
    smoother_ = std::make_unique<TrajectorySmoother>();

    raw_filename_ =
        "modules/tools/navi_generator/backend/util/testdata/"
        "trajectory_smoother/raw.txt";
    smoothed_filename_ =
        "modules/tools/navi_generator/backend/util/testdata/"
        "trajectory_smoother/processed.smoothed";
  }

 protected:
  std::unique_ptr<TrajectorySmoother> smoother_;
  std::string raw_filename_;
  std::string smoothed_filename_;
};

TEST_F(TrajectorySmootherTest, GoodData) {
  EXPECT_TRUE(smoother_->Import(raw_filename_));
  EXPECT_TRUE(smoother_->Smooth());
  EXPECT_TRUE(smoother_->Export(smoothed_filename_));
  EXPECT_GT(smoother_->smoothed_points().size(), 0);
}

TEST_F(TrajectorySmootherTest, EmptyData) {
  EXPECT_FALSE(smoother_->Import("foo.txt"));
  EXPECT_FALSE(smoother_->Smooth());
  EXPECT_FALSE(smoother_->Export("foo.smoothed"));
  EXPECT_EQ(0, smoother_->smoothed_points().size());
}

}  // namespace util
}  // namespace navi_generator
}  // namespace apollo
