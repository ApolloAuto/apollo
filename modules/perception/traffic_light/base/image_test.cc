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
#include <gtest/gtest.h>

#include "modules/perception/traffic_light/base/image.h"

namespace apollo {
namespace perception {
namespace traffic_light {

class ImageTest : public ::testing::Test {
 public:
  ImageTest() {}
  virtual ~ImageTest() {}

 protected:
  void SetUp() override { image_ = new Image(); }

  void TearDown() override { delete image_; }

 protected:
  Image *image_;
};

TEST_F(ImageTest, test_all) {
  cv::Mat img;
  CameraId cam_id = CameraId::LONG_FOCUS;
  double timestamp = 0.0;
  {
    std::stringstream ss;
    ss << *image_;
    EXPECT_EQ("Image not inited.", ss.str());
  }

  ASSERT_TRUE(image_->Init(timestamp, cam_id, img));
  {
    std::stringstream ss;
    ss << *image_;
    ASSERT_NE("Image not inited.", ss.str());
  }

  EXPECT_EQ(CameraId::LONG_FOCUS, image_->camera_id());
  EXPECT_DOUBLE_EQ(0.0, image_->ts());
  EXPECT_EQ("long_focus_camera_25mm", image_->camera_id_str());

  cam_id = CameraId::UNKNOWN;
  ASSERT_TRUE(image_->Init(timestamp, cam_id, img));
  EXPECT_EQ("unkown camera", image_->camera_id_str());
}
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
