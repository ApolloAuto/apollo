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
#include "modules/perception/common/inference/tensorrt/batch_stream.h"

#include "gtest/gtest.h"

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace inference {

TEST(BatchStreamTest, test_init) {
  {
    BatchStream batch_stream;
    batch_stream.reset(0);
    EXPECT_EQ(batch_stream.getBatchSize(), 0);
    EXPECT_EQ(batch_stream.getBatchesRead(), 0);
  }

  {
    BatchStream batch_stream(
        0, 0,
        "modules/perception/common/inference/inference_test_data/tensorrt/nonexists"); // NOLINT
    EXPECT_EQ(batch_stream.getBatchSize(), 0);
    EXPECT_EQ(batch_stream.getBatchesRead(), 0);
  }

  {
    BatchStream batch_stream(
        1, 5,
        "modules/perception/common/inference/inference_test_data/tensorrt/bs_1x1x1x1"); // NOLINT
    auto dims = batch_stream.getDims();
    batch_stream.reset(0);
    EXPECT_EQ(1, dims.n());
    EXPECT_EQ(1, dims.c());
    EXPECT_EQ(1, dims.h());
    EXPECT_EQ(1, dims.w());
  }
}

TEST(BatchStreamTest, test_update) {
  {
    BatchStream batch_stream(
        1, 3,
        "modules/perception/common/inference/inference_test_data/tensorrt/bs_1x1x1x1"); // NOLINT
    batch_stream.reset(0);
    EXPECT_TRUE(batch_stream.update());
    EXPECT_TRUE(batch_stream.update());
    EXPECT_TRUE(batch_stream.update());
    EXPECT_TRUE(batch_stream.update());
    EXPECT_FALSE(batch_stream.update());
    batch_stream.reset(0);
    EXPECT_TRUE(batch_stream.update());
  }
}

TEST(BatchStreamTest, test_next) {
  {
    BatchStream batch_stream(
        1, 3,
        "modules/perception/common/inference/inference_test_data/tensorrt/bs_1x1x1x1"); // NOLINT
    batch_stream.reset(0);
    EXPECT_TRUE(batch_stream.next());
    EXPECT_TRUE(batch_stream.next());
    EXPECT_TRUE(batch_stream.next());
    EXPECT_FALSE(batch_stream.next());
    batch_stream.reset(0);
    EXPECT_TRUE(batch_stream.next());
  }
  {
    BatchStream batch_stream(
        1, 5,
        "modules/perception/common/inference/inference_test_data/tensorrt/bs_1x1x1x1"); // NOLINT
    EXPECT_TRUE(batch_stream.next());
    EXPECT_TRUE(batch_stream.next());
    EXPECT_TRUE(batch_stream.next());
    EXPECT_TRUE(batch_stream.next());
    EXPECT_FALSE(batch_stream.next());
  }
}

TEST(BatchStreamTest, test_skip) {
  {
    BatchStream batch_stream(
        2, 2,
        "modules/perception/common/inference/inference_test_data/tensorrt/bs_1x1x1x1"); // NOLINT
    batch_stream.skip(1);
    EXPECT_EQ(batch_stream.mFileCount, 2);
    batch_stream.skip(1);
    EXPECT_EQ(batch_stream.mFileCount, 4);
  }
  {
    BatchStream batch_stream(
        1, 2,
        "modules/perception/common/inference/inference_test_data/tensorrt/bs_1x1x1x1"); // NOLINT
    batch_stream.skip(1);
    EXPECT_EQ(batch_stream.mFileCount, 1);
    batch_stream.skip(0);
    EXPECT_EQ(batch_stream.mFileCount, 1);
  }
  {
    BatchStream batch_stream(
        1, 6,
        "modules/perception/common/inference/inference_test_data/tensorrt/bs_2x1x1x1"); // NOLINT
    batch_stream.skip(1);
    EXPECT_EQ(batch_stream.mFileCount, 1);
    batch_stream.skip(1);
    EXPECT_EQ(batch_stream.mFileCount, 1);
  }
  {
    BatchStream batch_stream(
        3, 2,
        "modules/perception/common/inference/inference_test_data/tensorrt/bs_2x1x1x1"); // NOLINT
    batch_stream.skip(1);
    EXPECT_EQ(batch_stream.mFileCount, 2);
    batch_stream.skip(1);
    EXPECT_EQ(batch_stream.mFileCount, 3);
  }
}

}  // namespace inference
}  // namespace perception
}  // namespace apollo
