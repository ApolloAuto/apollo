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
#include "modules/perception/base/image.h"

namespace apollo {
namespace perception {
namespace base {

TEST(ImageTest, image8u_test) {
  {
    Image8U image;
    EXPECT_EQ(image.rows(), 0);
    EXPECT_EQ(image.cols(), 0);
    EXPECT_EQ(image.channels(), 0);
    EXPECT_EQ(image.width_step(), 0);
    EXPECT_EQ(image.total(), 0);
    EXPECT_EQ(image.type(), Color::NONE);
  }

  {
    std::shared_ptr<Blob<uint8_t>> blob(new Blob<uint8_t>({5, 5, 3}));
    for (int i = 0; i < blob->count(); ++i) {
      blob->mutable_cpu_data()[i] = i;
    }
    EXPECT_EQ(blob.use_count(), 1);

    {
      Image8U image(5, 5, Color::RGB, blob);
      EXPECT_EQ(image.cpu_ptr(), blob->cpu_data());
      EXPECT_EQ(image.gpu_ptr(), blob->gpu_data());

      EXPECT_EQ(image.rows(), 5);
      EXPECT_EQ(image.cols(), 5);
      EXPECT_EQ(image.channels(), 3);
      EXPECT_EQ(image.width_step(), 15);
      EXPECT_EQ(image.total(), 75);
      EXPECT_EQ(image.type(), Color::RGB);

      Rect<int> roi(0, 1, 2, 3);
      Image8U image_roi = image(roi);
      EXPECT_EQ(image_roi.cpu_ptr(0)[0], 15);
      EXPECT_EQ(image_roi.cpu_ptr(0)[1], 16);
      EXPECT_EQ(image_roi.cpu_ptr(1)[0], 30);
      EXPECT_EQ(image_roi.cpu_ptr(1)[1], 31);

      auto ref_blob = image.blob();
      EXPECT_EQ(blob.use_count(), 4);
      ref_blob = nullptr;

      EXPECT_EQ(blob.use_count(), 3);
      image = Image8U();
      EXPECT_EQ(blob.use_count(), 2);
      image_roi = Image8U();
      EXPECT_EQ(blob.use_count(), 1);
    }

    EXPECT_EQ(blob.use_count(), 1);
  }

  {
    Image8U image(5, 5, Color::RGB);
    EXPECT_EQ(image.blob().use_count(), 2);

    auto blob = image.blob();
    EXPECT_EQ(blob.use_count(), 2);
    for (int i = 0; i < blob->count(); ++i) {
      blob->mutable_cpu_data()[i] = i;
    }
    EXPECT_EQ(image.cpu_ptr(), blob->cpu_data());
    EXPECT_EQ(image.gpu_ptr(), blob->gpu_data());

    // copy and assign
    Image8U image_shared(image);
    EXPECT_EQ(blob.use_count(), 3);
    Image8U image_assigned = image;
    EXPECT_EQ(blob.use_count(), 4);
    image_shared = Image8U();
    image_assigned = Image8U();

    Rect<int> roi(0, 1, 2, 3);
    Image8U image_roi = image(roi);
    EXPECT_EQ(blob.use_count(), 3);
    EXPECT_EQ(image_roi.cpu_ptr(0)[0], 15);
    EXPECT_EQ(image_roi.cpu_ptr(0)[1], 16);
    EXPECT_EQ(image_roi.cpu_ptr(1)[0], 30);
    EXPECT_EQ(image_roi.cpu_ptr(1)[1], 31);

    auto ref_blob = image.blob();
    EXPECT_EQ(blob.use_count(), 4);
    ref_blob = nullptr;

    EXPECT_EQ(blob.use_count(), 3);
    image = Image8U();
    EXPECT_EQ(blob.use_count(), 2);
    image_roi = Image8U();
    EXPECT_EQ(blob.use_count(), 1);
  }

  {
    const Image8U image(5, 5, Color::RGB);
    auto blob = image.blob();
    EXPECT_EQ(blob.use_count(), 2);
    // blob->gpu_data();          // GOOD
    // blob->mutable_gpu_data();  // COMPILING ERROR
  }
}

}  // namespace base
}  // namespace perception
}  // namespace apollo
