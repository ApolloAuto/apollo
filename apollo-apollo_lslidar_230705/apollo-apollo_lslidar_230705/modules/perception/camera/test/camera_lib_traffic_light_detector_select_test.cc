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
#include "modules/perception/camera/lib/traffic_light/detector/detection/select.h"

#include "cyber/common/log.h"
#include "gtest/gtest.h"

namespace apollo {
namespace perception {
namespace camera {

TEST(MatchTest, init_test) {
  Select select;
  EXPECT_FALSE(select.Init(-1, 20));
  EXPECT_FALSE(select.Init(100, -3));
  EXPECT_TRUE(select.Init(10, 20));
}

TEST(MatchTest, nvn1) {
  Select select;
  std::vector<base::TrafficLightPtr> hdmap_bboxes;
  std::vector<base::TrafficLightPtr> detect_bboxes;

  select.Init(1000, 1000);
  {
    base::TrafficLightPtr light(new base::TrafficLight);
    light->region.detection_roi = base::RectI(100, 100, 10, 30);
    light->region.crop_roi = base::RectI(0, 0, 1920, 1080);
    hdmap_bboxes.push_back(light);
  }
  {
    base::TrafficLightPtr light(new base::TrafficLight);
    light->region.detection_roi = base::RectI(120, 140, 10, 30);
    light->region.is_detected = true;
    detect_bboxes.push_back(light);
  }

  select.SelectTrafficLights(detect_bboxes, &hdmap_bboxes);

  EXPECT_TRUE(hdmap_bboxes[0]->region.is_selected);
  EXPECT_TRUE(hdmap_bboxes[0]->region.is_detected);
  EXPECT_EQ(hdmap_bboxes[0]->region.detection_roi,
            detect_bboxes[0]->region.detection_roi);
}

TEST(MatchTest, nvn2) {
  Select select;
  std::vector<base::TrafficLightPtr> hdmap_bboxes;
  std::vector<base::TrafficLightPtr> detect_bboxes;

  select.Init(1000, 1000);
  {
    base::TrafficLightPtr light(new base::TrafficLight);
    light->region.detection_roi = base::RectI(100, 100, 10, 30);
    light->region.crop_roi = base::RectI(0, 0, 1920, 1080);
    light->region.detect_score = 0.9f;
    hdmap_bboxes.push_back(light);
  }
  {
    base::TrafficLightPtr light(new base::TrafficLight);
    light->region.detection_roi = base::RectI(160, 100, 10, 30);
    light->region.crop_roi = base::RectI(0, 0, 1920, 1080);
    light->region.detect_score = 0.9f;
    hdmap_bboxes.push_back(light);
  }
  {
    base::TrafficLightPtr light(new base::TrafficLight);
    light->region.detection_roi = base::RectI(220, 240, 10, 30);
    light->region.detect_score = 0.9f;
    light->region.is_detected = true;
    detect_bboxes.push_back(light);
  }
  {
    base::TrafficLightPtr light(new base::TrafficLight);
    light->region.detection_roi = base::RectI(240, 240, 10, 30);
    light->region.detect_score = 0.9f;
    light->region.is_detected = true;
    detect_bboxes.push_back(light);
  }

  select.SelectTrafficLights(detect_bboxes, &hdmap_bboxes);
  EXPECT_EQ(hdmap_bboxes.size(), 2);
  EXPECT_TRUE(hdmap_bboxes[0]->region.is_selected);
  EXPECT_TRUE(hdmap_bboxes[0]->region.is_detected);
  EXPECT_TRUE(hdmap_bboxes[1]->region.is_selected);
  EXPECT_TRUE(hdmap_bboxes[1]->region.is_detected);
  EXPECT_EQ(hdmap_bboxes[0]->region.detection_roi,
            detect_bboxes[0]->region.detection_roi);
  EXPECT_EQ(hdmap_bboxes[1]->region.detection_roi,
            detect_bboxes[1]->region.detection_roi);
}

TEST(MatchTest, nvm12) {
  Select select;
  std::vector<base::TrafficLightPtr> hdmap_bboxes;
  std::vector<base::TrafficLightPtr> detect_bboxes;

  select.Init(1000, 1000);
  {
    base::TrafficLightPtr light(new base::TrafficLight);
    light->region.detect_score = 0.9f;
    light->region.detection_roi = base::RectI(100, 100, 10, 30);
    light->region.crop_roi = base::RectI(0, 0, 1920, 1080);
    hdmap_bboxes.push_back(light);
  }
  {
    base::TrafficLightPtr light(new base::TrafficLight);
    light->region.detect_score = 0.9f;
    light->region.detection_roi = base::RectI(220, 240, 10, 30);
    light->region.is_detected = true;
    detect_bboxes.push_back(light);
  }
  {
    base::TrafficLightPtr light(new base::TrafficLight);
    light->region.detect_score = 0.9f;
    light->region.detection_roi = base::RectI(170, 240, 10, 30);
    light->region.is_detected = true;
    detect_bboxes.push_back(light);
  }

  select.SelectTrafficLights(detect_bboxes, &hdmap_bboxes);

  EXPECT_EQ(hdmap_bboxes.size(), 1);
  EXPECT_TRUE(hdmap_bboxes[0]->region.is_selected);
  EXPECT_TRUE(hdmap_bboxes[0]->region.is_detected);
  EXPECT_EQ(hdmap_bboxes[0]->region.detection_roi,
            detect_bboxes[1]->region.detection_roi);
}

TEST(MatchTest, nvm21) {
  Select select;
  std::vector<base::TrafficLightPtr> hdmap_bboxes;
  std::vector<base::TrafficLightPtr> detect_bboxes;

  select.Init(1000, 1000);
  {
    base::TrafficLightPtr light(new base::TrafficLight);
    light->region.detect_score = 0.9f;
    light->region.detection_roi = base::RectI(100, 100, 10, 30);
    light->region.crop_roi = base::RectI(0, 0, 1920, 1080);
    hdmap_bboxes.push_back(light);
  }
  {
    base::TrafficLightPtr light(new base::TrafficLight);
    light->region.detect_score = 0.9f;
    light->region.detection_roi = base::RectI(160, 100, 10, 30);
    light->region.crop_roi = base::RectI(0, 0, 1920, 1080);
    hdmap_bboxes.push_back(light);
  }
  {
    base::TrafficLightPtr light(new base::TrafficLight);
    light->region.detect_score = 0.9f;
    light->region.detection_roi = base::RectI(120, 140, 10, 30);
    light->region.is_detected = true;
    detect_bboxes.push_back(light);
  }
  select.SelectTrafficLights(detect_bboxes, &hdmap_bboxes);

  EXPECT_EQ(hdmap_bboxes.size(), 2);
  EXPECT_TRUE(hdmap_bboxes[0]->region.is_selected);
  EXPECT_FALSE(hdmap_bboxes[1]->region.is_selected);
  EXPECT_TRUE(hdmap_bboxes[0]->region.is_detected);
  EXPECT_FALSE(hdmap_bboxes[1]->region.is_detected);
  EXPECT_EQ(hdmap_bboxes[0]->region.detection_roi,
            detect_bboxes[0]->region.detection_roi);
  EXPECT_EQ(hdmap_bboxes[1]->region.detection_roi,
            hdmap_bboxes[1]->region.detection_roi);
}

TEST(MatchTest, nvm24) {
  Select select;
  std::vector<base::TrafficLightPtr> hdmap_bboxes;
  std::vector<base::TrafficLightPtr> detect_bboxes;

  select.Init(1000, 1000);
  {
    base::TrafficLightPtr light(new base::TrafficLight);
    light->region.detect_score = 0.9f;
    light->region.detection_roi = base::RectI(100, 100, 10, 30);
    light->region.crop_roi = base::RectI(0, 0, 1920, 1080);
    hdmap_bboxes.push_back(light);
  }
  {
    base::TrafficLightPtr light(new base::TrafficLight);
    light->region.detect_score = 0.9f;
    light->region.detection_roi = base::RectI(160, 100, 10, 30);
    light->region.crop_roi = base::RectI(0, 0, 1920, 1080);
    hdmap_bboxes.push_back(light);
  }

  {
    base::TrafficLightPtr light(new base::TrafficLight);
    light->region.detect_score = 0.9f;
    light->region.detection_roi = base::RectI(120, 140, 10, 40);
    light->region.is_detected = true;
    detect_bboxes.push_back(light);
  }
  {
    base::TrafficLightPtr light(new base::TrafficLight);
    light->region.detect_score = 0.9f;
    light->region.detection_roi = base::RectI(160, 140, 10, 40);
    light->region.is_detected = true;
    detect_bboxes.push_back(light);
  }
  {
    base::TrafficLightPtr light(new base::TrafficLight);
    light->region.detect_score = 0.9f;
    light->region.detection_roi = base::RectI(130, 150, 20, 20);
    light->region.is_detected = true;
    detect_bboxes.push_back(light);
  }
  {
    base::TrafficLightPtr light(new base::TrafficLight);
    light->region.detect_score = 0.9f;
    light->region.detection_roi = base::RectI(170, 150, 20, 20);
    light->region.is_detected = true;
    detect_bboxes.push_back(light);
  }
  select.SelectTrafficLights(detect_bboxes, &hdmap_bboxes);

  EXPECT_EQ(hdmap_bboxes.size(), 2);
  EXPECT_TRUE(hdmap_bboxes[0]->region.is_selected);
  EXPECT_TRUE(hdmap_bboxes[1]->region.is_selected);
  EXPECT_TRUE(hdmap_bboxes[0]->region.is_detected);
  EXPECT_TRUE(hdmap_bboxes[1]->region.is_detected);
  EXPECT_EQ(hdmap_bboxes[0]->region.detection_roi,
            detect_bboxes[0]->region.detection_roi);
  EXPECT_EQ(hdmap_bboxes[1]->region.detection_roi,
            detect_bboxes[1]->region.detection_roi);
}

TEST(MatchTest, test_outside_crop_roi) {
  Select select;
  std::vector<base::TrafficLightPtr> hdmap_bboxes;
  std::vector<base::TrafficLightPtr> detect_bboxes;

  select.Init(1000, 1000);
  {
    base::TrafficLightPtr light(new base::TrafficLight);
    light->region.detection_roi = base::RectI(100, 100, 10, 30);
    light->region.crop_roi = base::RectI(50, 50, 100, 100);
    light->region.detect_score = 0.9f;
    hdmap_bboxes.push_back(light);
  }
  {
    base::TrafficLightPtr light(new base::TrafficLight);
    light->region.detection_roi = base::RectI(160, 100, 10, 30);
    light->region.crop_roi = base::RectI(110, 50, 100, 100);
    light->region.detect_score = 0.9f;
    hdmap_bboxes.push_back(light);
  }
  {
    base::TrafficLightPtr light(new base::TrafficLight);
    light->region.detection_roi = base::RectI(220, 240, 10, 30);
    light->region.detect_score = 0.9f;
    light->region.is_detected = true;
    detect_bboxes.push_back(light);
  }
  {
    base::TrafficLightPtr light(new base::TrafficLight);
    light->region.detection_roi = base::RectI(240, 240, 10, 30);
    light->region.detect_score = 0.9f;
    light->region.is_detected = true;
    detect_bboxes.push_back(light);
  }

  select.SelectTrafficLights(detect_bboxes, &hdmap_bboxes);
  EXPECT_EQ(hdmap_bboxes.size(), 2);
  EXPECT_TRUE(hdmap_bboxes[0]->region.is_selected);
  EXPECT_TRUE(hdmap_bboxes[1]->region.is_selected);
  EXPECT_FALSE(hdmap_bboxes[0]->region.is_detected);
  EXPECT_FALSE(hdmap_bboxes[1]->region.is_detected);
}

TEST(MatchTest, test_outside_image) {
  Select select;
  std::vector<base::TrafficLightPtr> hdmap_bboxes;
  std::vector<base::TrafficLightPtr> detect_bboxes;

  select.Init(1000, 1000);
  {
    base::TrafficLightPtr light(new base::TrafficLight);
    light->region.outside_image = true;
    light->region.detection_roi = base::RectI(100, 100, 10, 30);
    light->region.crop_roi = base::RectI(0, 0, 1920, 1080);
    light->region.detect_score = 0.9f;
    hdmap_bboxes.push_back(light);
  }
  {
    base::TrafficLightPtr light(new base::TrafficLight);
    light->region.detection_roi = base::RectI(160, 100, 10, 30);
    light->region.crop_roi = base::RectI(0, 0, 1920, 1080);
    light->region.detect_score = 0.9f;
    hdmap_bboxes.push_back(light);
  }
  {
    base::TrafficLightPtr light(new base::TrafficLight);
    light->region.detection_roi = base::RectI(220, 240, 10, 30);
    light->region.detect_score = 0.9f;
    light->region.is_detected = true;
    detect_bboxes.push_back(light);
  }
  {
    base::TrafficLightPtr light(new base::TrafficLight);
    light->region.detection_roi = base::RectI(240, 240, 10, 30);
    light->region.detect_score = 0.9f;
    light->region.is_detected = true;
    detect_bboxes.push_back(light);
  }

  select.SelectTrafficLights(detect_bboxes, &hdmap_bboxes);
  EXPECT_EQ(hdmap_bboxes.size(), 2);
  EXPECT_TRUE(hdmap_bboxes[0]->region.is_selected);
  EXPECT_TRUE(hdmap_bboxes[1]->region.is_selected);
  EXPECT_FALSE(hdmap_bboxes[0]->region.is_detected);
  EXPECT_TRUE(hdmap_bboxes[1]->region.is_detected);
  EXPECT_EQ(hdmap_bboxes[1]->region.detection_roi,
            detect_bboxes[0]->region.detection_roi);
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
