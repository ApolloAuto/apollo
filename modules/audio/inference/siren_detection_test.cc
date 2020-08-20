/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/audio/inference/siren_detection.h"

#include "gtest/gtest.h"

namespace apollo {
namespace audio {

class SirenDetectionTest : public ::testing::Test {
 public:
  virtual void SetUp() {}

 protected:
  SirenDetection siren_detection_;
};

TEST_F(SirenDetectionTest, is_siren) {
  std::vector<std::vector<double>> signals(4,
    (std::vector<double> (72000, 0.01)));
  bool result = siren_detection_.Evaluate(signals);
  EXPECT_EQ(result, false);
}

}  // namespace audio
}  // namespace apollo
