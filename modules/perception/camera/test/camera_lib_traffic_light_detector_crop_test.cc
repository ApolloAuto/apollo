/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/camera/lib/traffic_light/detector/detection/cropbox.h"

#include "cyber/common/log.h"
#include "gtest/gtest.h"

namespace apollo {
namespace perception {
namespace camera {

TEST(CropBoxTest, wholebox) {
  int img_width = 1920;
  int img_height = 1080;
  {
    base::RectI gt_box(0, 0, 1920, 1080);
    std::shared_ptr<IGetBox> crop;
    crop.reset(new CropBoxWholeImage());
    base::TrafficLightPtr light;
    base::RectI project_roi(1, 2, 3, 4);
    light.reset(new base::TrafficLight);
    light->region.projection_roi = project_roi;
    base::RectI cbox;
    crop->getCropBox(img_width, img_height, light, &cbox);
    EXPECT_EQ(cbox, gt_box);
  }

  {
    base::RectI gt_box(0, 0, 0, 0);
    std::shared_ptr<IGetBox> crop;
    crop.reset(new CropBoxWholeImage());
    base::TrafficLightPtr light;
    base::RectI project_roi(1, -2, 3, 4);
    light.reset(new base::TrafficLight);
    light->region.projection_roi = project_roi;
    base::RectI cbox;
    crop->getCropBox(img_width, img_height, light, &cbox);
    EXPECT_EQ(cbox, gt_box);
  }

  {
    base::RectI gt_box(0, 0, 0, 0);
    std::shared_ptr<IGetBox> crop;
    crop.reset(new CropBoxWholeImage());
    base::TrafficLightPtr light;
    base::RectI project_roi(1, -2, 0, 4);
    light.reset(new base::TrafficLight);
    light->region.projection_roi = project_roi;
    base::RectI cbox;
    crop->getCropBox(img_width, img_height, light, &cbox);
    EXPECT_EQ(cbox, gt_box);
  }

  {
    base::RectI gt_box(0, 0, 0, 0);
    std::shared_ptr<IGetBox> crop;
    crop.reset(new CropBoxWholeImage());
    base::TrafficLightPtr light;
    base::RectI project_roi(1, 1, 0, 0);
    light.reset(new base::TrafficLight);
    light->region.projection_roi = project_roi;
    base::RectI cbox;
    crop->getCropBox(img_width, img_height, light, &cbox);
    EXPECT_EQ(cbox, gt_box);
  }
}

TEST(CropBoxTest, crop_roi) {
  int img_width = 1920;
  int img_height = 1080;
  std::shared_ptr<IGetBox> crop;
  crop.reset(new CropBox(2.5, 210));
  base::TrafficLightPtr light;
  light.reset(new base::TrafficLight);
  base::RectI cbox;

  // normal
  {
    base::RectI gt_box(25, 25, 250, 250);
    base::RectI project_roi(100, 100, 100, 100);
    light->region.projection_roi = project_roi;
    crop->getCropBox(img_width, img_height, light, &cbox);
    EXPECT_EQ(cbox, gt_box);
  }

  // invalid roi
  {
    base::RectI gt_box(0, 0, 0, 0);
    base::RectI project_roi(100, 100, 500, 5000);
    light->region.projection_roi = project_roi;
    crop->getCropBox(img_width, img_height, light, &cbox);
    EXPECT_EQ(cbox, gt_box);
  }

  // invalid roi
  {
    base::RectI gt_box(0, 0, 0, 0);
    base::RectI project_roi(100, 100, 500, 0);
    light->region.projection_roi = project_roi;
    crop->getCropBox(img_width, img_height, light, &cbox);
    EXPECT_EQ(cbox, gt_box);
  }

  // out of boundary
  {
    base::RectI gt_box(0, 0, 1080, 1080);
    base::RectI project_roi(100, 100, 500, 500);
    light->region.projection_roi = project_roi;
    crop->getCropBox(img_width, img_height, light, &cbox);
    EXPECT_EQ(cbox, gt_box);
  }

  // out of boundary
  {
    base::RectI gt_box(840, 0, 1080, 1080);
    base::RectI project_roi(1419, 579, 500, 500);
    light->region.projection_roi = project_roi;
    crop->getCropBox(img_width, img_height, light, &cbox);
    EXPECT_EQ(cbox, gt_box);
  }
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
