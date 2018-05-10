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

#include "modules/prediction/common/feature_output.h"

#include "gtest/gtest.h"

namespace apollo {
namespace prediction {

class FeatureOutputTest : public ::testing::Test {
 public:
  void SetUp() override { FeatureOutput::Ready(); }
};

TEST_F(FeatureOutputTest, get_ready) {
  EXPECT_TRUE(FeatureOutput::Ready());
  EXPECT_EQ(0, FeatureOutput::Size());
}

TEST_F(FeatureOutputTest, insertion) {
  Feature feature;
  for (int i = 0; i < 3; ++i) {
    Feature feature;
    FeatureOutput::Insert(feature);
  }
  EXPECT_EQ(3, FeatureOutput::Size());
}

TEST_F(FeatureOutputTest, clear) {
  Feature feature;
  for (int i = 0; i < 3; ++i) {
    Feature feature;
    FeatureOutput::Insert(feature);
  }
  FeatureOutput::Clear();
  EXPECT_EQ(0, FeatureOutput::Size());
}

}  // namespace prediction
}  // namespace apollo
