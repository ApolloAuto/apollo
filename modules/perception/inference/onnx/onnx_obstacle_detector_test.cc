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
#include "modules/perception/inference/onnx/onnx_obstacle_detector.h"

#include "gtest/gtest.h"

#include "modules/perception/common/perception_gflags.h"

namespace apollo {
namespace perception {
namespace inference {

TEST(OnnxInferenceTest, test) {
  OnnxObstacleDetector test_onnx_detector(
    FLAGS_onnx_obstacle_detector_model,
    FLAGS_num_classes,
    FLAGS_onnx_test_input_path,
    FLAGS_onnx_test_input_name_file,
    FLAGS_onnx_prediction_image_path);
  test_onnx_detector.Infer();
  int dummy = 1;
  EXPECT_EQ(dummy, 1);
}

} // namespace inference
} // namespace perception
} // namespace apollo