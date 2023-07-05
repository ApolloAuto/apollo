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

#include "modules/audio/inference/moving_detection.h"

#include "gtest/gtest.h"

namespace apollo {
namespace audio {

class MovingDetectionTest : public ::testing::Test {
 public:
  virtual void SetUp() {}

 protected:
  MovingDetection moving_detection_;
};

TEST_F(MovingDetectionTest, fft1d) {
  std::vector<double> signal{1.0, 2.0, 3.0, 4.0, -1.0, -2.0};
  std::vector<std::complex<double>> fft_result =
      moving_detection_.fft1d(signal);
}

TEST_F(MovingDetectionTest, moving) {
  std::vector<std::vector<double>> signals{
    {1.0, 2.0, 3.0, 4.0, -1.0, -2.0},
    {-1.0, -2.0, 3.0, 4.0, 1.0, -2.0}};
  MovingResult result = moving_detection_.Detect(signals);
  EXPECT_EQ(result, UNKNOWN);
}

}  // namespace audio
}  // namespace apollo
