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
#include "modules/perception/inference/onnx/libtorch_obstacle_detector.h"

#include "gtest/gtest.h"

#include "modules/perception/common/perception_gflags.h"

namespace apollo {
namespace perception {
namespace inference {

class LibtorchObstacleDetectionTest : public ::testing::Test {
 public:
  virtual void SetUp() {}

 protected:
  LibtorchObstacleDetection obstacle_detection_;
};

TEST_F(LibtorchObstacleDetectionTest, is_) {
  std::vector<std::vector<std::vector<double> > >imageFrame(3,
    std::vector<std::vector<double> >(608,
    std::vector<double>(608, 0.01)));
  bool result = obstacle_detection_.Evaluate(imageFrame);
  EXPECT_EQ(result, false);
}

}  // namespace inference
}  // namespace perception
}  // namespace apollo
