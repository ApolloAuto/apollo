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

#include "modules/perception/common/inference/tensorrt/entropy_calibrator.h"

#include "gtest/gtest.h"

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace inference {

TEST(Int8EntropyCalibratorTest, test_init) {
  // test empty batch_stream
  {
    BatchStream batch_stream;
    nvinfer1::Int8EntropyCalibrator calibrator(batch_stream, 0, false, "");
    EXPECT_EQ(calibrator.getBatchSize(), 0);
  }
  // test valid batch_stream
  {
    BatchStream batch_stream(
        1, 5,
        "modules/perception/common/inference/inference_test_data/tensorrt/bs_1x1x1x1"); // NOLINT
    nvinfer1::Int8EntropyCalibrator calibrator(
        batch_stream, 0, false,
        "modules/perception/common/inference/inference_test_data/tensorrt");
    EXPECT_EQ(calibrator.getBatchSize(), 1);
  }
}

TEST(Int8EntropyCalibratorTest, test_cache) {
  // DONOT read cache
  {
    BatchStream batch_stream(
        1, 5,
        "modules/perception/common/inference/inference_test_data/tensorrt/bs_1x1x1x1"); // NOLINT
    nvinfer1::Int8EntropyCalibrator calibrator(
        batch_stream, 0, false,
        "modules/perception/common/inference/inference_test_data/tensorrt");
    size_t length;
    auto cache = calibrator.readCalibrationCache(length);
    EXPECT_EQ(length, 0);
    EXPECT_EQ(cache, nullptr);
  }

  // DO read cache from BAD path
  {
    BatchStream batch_stream(
        1, 5,
        "modules/perception/common/inference/inference_test_data/tensorrt/bs_1x1x1x1"); // NOLINT
    nvinfer1::Int8EntropyCalibrator calibrator(
        batch_stream, 0, true,
        "modules/perception/common/inference/inference_test_data/tensorrt-nonexists"); // NOLINT
    size_t length;
    auto cache = calibrator.readCalibrationCache(length);
    EXPECT_EQ(length, 0);
    EXPECT_EQ(cache, nullptr);
  }

  // DO read cache
  {
    BatchStream batch_stream(
        1, 5,
        "modules/perception/common/inference/inference_test_data/tensorrt/bs_1x1x1x1"); // NOLINT
    nvinfer1::Int8EntropyCalibrator calibrator(
        batch_stream, 0, true,
        "modules/perception/common/inference/inference_test_data/tensorrt");
    size_t length;
    auto cache = calibrator.readCalibrationCache(length);
    EXPECT_EQ(length, 1128);
    EXPECT_NE(cache, nullptr);
  }
}

TEST(Int8EntropyCalibratorTest, test_get_batch) {
  // DO read cache
  {
    BatchStream batch_stream(
        1, 5,
        "modules/perception/common/inference/inference_test_data/tensorrt/bs_1x1x1x1"); // NOLINT
    nvinfer1::Int8EntropyCalibrator calibrator(
        batch_stream, 0, true,
        "modules/perception/common/inference/inference_test_data/tensorrt");
    size_t length;
    auto cache = calibrator.readCalibrationCache(length);
    EXPECT_EQ(length, 1128);
    EXPECT_NE(cache, nullptr);
    void *bindings[1];
    const char *names[] = {"haha"};
    int nbBindings = 1;
    EXPECT_TRUE(calibrator.getBatch(bindings, names, nbBindings));
    EXPECT_TRUE(calibrator.getBatch(bindings, names, nbBindings));
    EXPECT_TRUE(calibrator.getBatch(bindings, names, nbBindings));
    EXPECT_TRUE(calibrator.getBatch(bindings, names, nbBindings));
    EXPECT_FALSE(calibrator.getBatch(bindings, names, nbBindings));
  }
}

}  // namespace inference
}  // namespace perception
}  // namespace apollo
