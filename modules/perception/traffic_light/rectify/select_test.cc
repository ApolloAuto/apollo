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
#include "modules/perception/traffic_light/rectify/select.h"

#include "gtest/gtest.h"

namespace apollo {
namespace perception {
namespace traffic_light {

class MatchTest : public ::testing::Test {
 protected:
  virtual void SetUp() {}
  ~MatchTest() {}

 protected:
  GaussianSelect select;
};

TEST_F(MatchTest, nvn1) {
  cv::Mat img;
  std::vector<LightPtr> hdmap_bboxes;
  std::vector<LightPtr> detect_bboxes;
  std::vector<LightPtr> selected_bboxes;
  {
    LightPtr light(new Light);
    light->region.rectified_roi = cv::Rect(100, 100, 10, 30);
    hdmap_bboxes.push_back(light);
  }
  {
    LightPtr light(new Light);
    light->region.rectified_roi = cv::Rect(120, 140, 10, 30);
    detect_bboxes.push_back(light);
  }
  select.Select(img, hdmap_bboxes, detect_bboxes, &selected_bboxes);

  ASSERT_TRUE(selected_bboxes[0]->region.is_selected);
}

TEST_F(MatchTest, nvn2) {
  cv::Mat img;
  std::vector<LightPtr> hdmap_bboxes;
  std::vector<LightPtr> detect_bboxes;
  std::vector<LightPtr> selected_bboxes;
  {
    LightPtr light(new Light);
    light->region.rectified_roi = cv::Rect(100, 100, 10, 30);
    light->region.detect_score = 0.9;
    hdmap_bboxes.push_back(light);
  }
  {
    LightPtr light(new Light);
    light->region.rectified_roi = cv::Rect(160, 100, 10, 30);
    light->region.detect_score = 0.9;
    hdmap_bboxes.push_back(light);
  }
  {
    LightPtr light(new Light);
    light->region.rectified_roi = cv::Rect(220, 240, 10, 30);
    light->region.detect_score = 0.9;
    detect_bboxes.push_back(light);
  }
  {
    LightPtr light(new Light);
    light->region.rectified_roi = cv::Rect(240, 240, 10, 30);
    light->region.detect_score = 0.9;
    detect_bboxes.push_back(light);
  }

  select.Select(img, hdmap_bboxes, detect_bboxes, &selected_bboxes);
  ASSERT_EQ(selected_bboxes.size(), 2);
  ASSERT_TRUE(selected_bboxes[0]->region.is_selected);
  ASSERT_TRUE(selected_bboxes[1]->region.is_selected);
  ASSERT_EQ(selected_bboxes[0]->region.rectified_roi,
            detect_bboxes[0]->region.rectified_roi);
  ASSERT_EQ(selected_bboxes[1]->region.rectified_roi,
            detect_bboxes[1]->region.rectified_roi);
}

TEST_F(MatchTest, nvm12) {
  cv::Mat img;
  std::vector<LightPtr> hdmap_bboxes;
  std::vector<LightPtr> detect_bboxes;
  std::vector<LightPtr> selected_bboxes;
  {
    LightPtr light(new Light);
    light->region.detect_score = 0.9;
    light->region.rectified_roi = cv::Rect(100, 100, 10, 30);
    hdmap_bboxes.push_back(light);
  }
  {
    LightPtr light(new Light);
    light->region.detect_score = 0.9;
    light->region.rectified_roi = cv::Rect(220, 240, 10, 30);
    detect_bboxes.push_back(light);
  }
  {
    LightPtr light(new Light);
    light->region.detect_score = 0.9;
    light->region.rectified_roi = cv::Rect(170, 240, 10, 30);
    detect_bboxes.push_back(light);
  }

  select.Select(img, hdmap_bboxes, detect_bboxes, &selected_bboxes);

  ASSERT_EQ(selected_bboxes.size(), 1);
  ASSERT_TRUE(selected_bboxes[0]->region.is_selected);
  std::cout << selected_bboxes[0]->region.rectified_roi << std::endl;
  std::cout << detect_bboxes[1]->region.rectified_roi << std::endl;
  ASSERT_EQ(selected_bboxes[0]->region.rectified_roi,
            detect_bboxes[1]->region.rectified_roi);

  hdmap_bboxes[0]->region.rectified_roi.x = 220;
  selected_bboxes.clear();
  select.Select(img, hdmap_bboxes, detect_bboxes, &selected_bboxes);

  ASSERT_EQ(selected_bboxes.size(), 1);
  ASSERT_TRUE(selected_bboxes[0]->region.is_selected);
  ASSERT_EQ(selected_bboxes[0]->region.rectified_roi,
            detect_bboxes[0]->region.rectified_roi);
}

TEST_F(MatchTest, nvm21) {
  cv::Mat img;
  std::vector<LightPtr> hdmap_bboxes;
  std::vector<LightPtr> detect_bboxes;
  std::vector<LightPtr> selected_bboxes;
  {
    LightPtr light(new Light);
    light->region.detect_score = 0.9;
    light->region.rectified_roi = cv::Rect(100, 100, 10, 30);
    hdmap_bboxes.push_back(light);
  }
  {
    LightPtr light(new Light);
    light->region.detect_score = 0.9;
    light->region.rectified_roi = cv::Rect(160, 100, 10, 30);
    hdmap_bboxes.push_back(light);
  }
  {
    LightPtr light(new Light);
    light->region.detect_score = 0.9;
    light->region.rectified_roi = cv::Rect(120, 140, 10, 30);
    detect_bboxes.push_back(light);
  }
  select.Select(img, hdmap_bboxes, detect_bboxes, &selected_bboxes);

  ASSERT_EQ(selected_bboxes.size(), 2);
  ASSERT_TRUE(selected_bboxes[0]->region.is_selected);
  ASSERT_FALSE(selected_bboxes[1]->region.is_selected);
  ASSERT_EQ(selected_bboxes[0]->region.rectified_roi,
            detect_bboxes[0]->region.rectified_roi);
  ASSERT_EQ(selected_bboxes[1]->region.rectified_roi,
            hdmap_bboxes[1]->region.rectified_roi);
}

TEST_F(MatchTest, nvm24) {
  cv::Mat img;
  std::vector<LightPtr> hdmap_bboxes;
  std::vector<LightPtr> detect_bboxes;
  std::vector<LightPtr> selected_bboxes;
  {
    LightPtr light(new Light);
    light->region.detect_score = 0.9;
    light->region.rectified_roi = cv::Rect(100, 100, 10, 30);
    hdmap_bboxes.push_back(light);
  }
  {
    LightPtr light(new Light);
    light->region.detect_score = 0.9;
    light->region.rectified_roi = cv::Rect(160, 100, 10, 30);
    hdmap_bboxes.push_back(light);
  }

  {
    LightPtr light(new Light);
    light->region.detect_score = 0.9;
    light->region.rectified_roi = cv::Rect(120, 140, 10, 40);
    detect_bboxes.push_back(light);
  }
  {
    LightPtr light(new Light);
    light->region.detect_score = 0.9;
    light->region.rectified_roi = cv::Rect(160, 140, 10, 40);
    detect_bboxes.push_back(light);
  }
  {
    LightPtr light(new Light);
    light->region.detect_score = 0.9;
    light->region.rectified_roi = cv::Rect(130, 150, 20, 20);
    detect_bboxes.push_back(light);
  }
  {
    LightPtr light(new Light);
    light->region.detect_score = 0.9;
    light->region.rectified_roi = cv::Rect(170, 150, 20, 20);
    detect_bboxes.push_back(light);
  }
  select.Select(img, hdmap_bboxes, detect_bboxes, &selected_bboxes);

  ASSERT_EQ(selected_bboxes.size(), 2);
  ASSERT_TRUE(selected_bboxes[0]->region.is_selected);
  ASSERT_TRUE(selected_bboxes[1]->region.is_selected);
  ASSERT_EQ(selected_bboxes[0]->region.rectified_roi,
            detect_bboxes[0]->region.rectified_roi);
  ASSERT_EQ(selected_bboxes[1]->region.rectified_roi,
            detect_bboxes[1]->region.rectified_roi);
}

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
