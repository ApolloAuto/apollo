//
// Created by gaohan02 on 16-12-14.
//

#include <traffic_light/rectify/unity/select.h>
#include "gtest/gtest.h"

namespace adu {
namespace perception {
namespace traffic_light {

class MatchTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
  }
  ~MatchTest() {
  }
 protected:
  Select select;
};

TEST_F(MatchTest, nvn1) {
  cv::Mat img;
  std::vector<LightPtr> hdmap_bboxes;
  std::vector<LightPtr> detect_bboxes;
  std::vector<LightPtr> selected_bboxes;
  LightPtr light(new Light);
  light->region.rectified_roi = cv::Rect(100, 100, 10, 30);
  hdmap_bboxes.push_back(light);

  light->region.rectified_roi.x += 20;
  light->region.rectified_roi.y += 40;
  detect_bboxes.push_back(light);
  select.select(img, hdmap_bboxes, detect_bboxes, &selected_bboxes);

  ASSERT_TRUE(selected_bboxes[0]->region.is_selected);
}

TEST_F(MatchTest, nvn2) {
  cv::Mat img;
  std::vector<LightPtr> hdmap_bboxes;
  std::vector<LightPtr> detect_bboxes;
  std::vector<LightPtr> selected_bboxes;
  LightPtr light(new Light);
  light->region.rectified_roi = cv::Rect(100, 100, 10, 30);
  light->region.detect_score = 0.9;
  hdmap_bboxes.push_back(light);
  light->region.rectified_roi.x += 60;
  hdmap_bboxes.push_back(light);

  light->region.rectified_roi = cv::Rect(220, 240, 10, 30);

  detect_bboxes.push_back(light);
  light->region.rectified_roi.x += 20;
  detect_bboxes.push_back(light);
  select.select(img, hdmap_bboxes, detect_bboxes, &selected_bboxes);
  ASSERT_EQ(selected_bboxes.size(), 2);
  ASSERT_TRUE(selected_bboxes[0]->region.is_selected);
  ASSERT_TRUE(selected_bboxes[1]->region.is_selected);
  ASSERT_EQ(selected_bboxes[0]->region.rectified_roi, detect_bboxes[0]->region.rectified_roi);
  ASSERT_EQ(selected_bboxes[1]->region.rectified_roi, detect_bboxes[1]->region.rectified_roi);
}

TEST_F(MatchTest, nvm12) {
  cv::Mat img;
  std::vector<LightPtr> hdmap_bboxes;
  std::vector<LightPtr> detect_bboxes;
  std::vector<LightPtr> selected_bboxes;
  LightPtr light(new Light);
  light->region.detect_score = 0.9;
  light->region.rectified_roi = cv::Rect(100, 100, 10, 30);
  hdmap_bboxes.push_back(light);

  light->region.rectified_roi = cv::Rect(220, 240, 10, 30);
  detect_bboxes.push_back(light);
  light->region.rectified_roi.x -= 50;
  detect_bboxes.push_back(light);
  select.select(img, hdmap_bboxes, detect_bboxes, &selected_bboxes);

  ASSERT_EQ(selected_bboxes.size(), 1);
  ASSERT_TRUE(selected_bboxes[0]->region.is_selected);
  std::cout << selected_bboxes[0]->region.rectified_roi << std::endl;
  std::cout << detect_bboxes[1]->region.rectified_roi << std::endl;
  ASSERT_TRUE(
      selected_bboxes[0]->region.rectified_roi == detect_bboxes[1]->region.rectified_roi);

  hdmap_bboxes[0]->region.rectified_roi.x = 220;
  selected_bboxes.clear();
  select.select(img, hdmap_bboxes, detect_bboxes, &selected_bboxes);

  ASSERT_EQ(selected_bboxes.size(), 1);
  ASSERT_TRUE(selected_bboxes[0]->region.is_selected);
  ASSERT_TRUE(
      selected_bboxes[0]->region.rectified_roi == detect_bboxes[0]->region.rectified_roi);
}

TEST_F(MatchTest, nvm21) {
  cv::Mat img;
  std::vector<LightPtr> hdmap_bboxes;
  std::vector<LightPtr> detect_bboxes;
  std::vector<LightPtr> selected_bboxes;
  LightPtr light(new Light);
  light->region.detect_score = 0.9;
  light->region.rectified_roi = cv::Rect(100, 100, 10, 30);
  hdmap_bboxes.push_back(light);
  light->region.rectified_roi.x += 60;
  hdmap_bboxes.push_back(light);

  light->region.rectified_roi = cv::Rect(120, 140, 10, 30);
  detect_bboxes.push_back(light);
  select.select(img, hdmap_bboxes, detect_bboxes, &selected_bboxes);

  ASSERT_EQ(selected_bboxes.size(), 2);
  ASSERT_TRUE(selected_bboxes[0]->region.is_selected);
  ASSERT_FALSE(selected_bboxes[1]->region.is_selected);
  ASSERT_TRUE(
      selected_bboxes[0]->region.rectified_roi == detect_bboxes[0]->region.rectified_roi);
  ASSERT_TRUE(selected_bboxes[1]->region.rectified_roi == hdmap_bboxes[1]->region.rectified_roi);
}

TEST_F(MatchTest, nvm24) {
  cv::Mat img;
  std::vector<LightPtr> hdmap_bboxes;
  std::vector<LightPtr> detect_bboxes;
  std::vector<LightPtr> selected_bboxes;
  LightPtr light(new Light);
  light->region.detect_score = 0.9;
  light->region.rectified_roi = cv::Rect(100, 100, 10, 30);
  hdmap_bboxes.push_back(light);
  light->region.rectified_roi.x += 60;
  hdmap_bboxes.push_back(light);

  light->region.rectified_roi = cv::Rect(120, 140, 10, 40);
  detect_bboxes.push_back(light);
  light->region.rectified_roi.x += 40;
  detect_bboxes.push_back(light);
  light->region.rectified_roi = cv::Rect(130, 150, 20, 20);
  detect_bboxes.push_back(light);
  light->region.rectified_roi.x += 40;
  detect_bboxes.push_back(light);

  select.select(img, hdmap_bboxes, detect_bboxes, &selected_bboxes);

  ASSERT_EQ(selected_bboxes.size(), 2);
  ASSERT_TRUE(selected_bboxes[0]->region.is_selected);
  ASSERT_TRUE(selected_bboxes[1]->region.is_selected);
  ASSERT_TRUE(selected_bboxes[0]->region.rectified_roi == detect_bboxes[0]->region.rectified_roi);
  ASSERT_TRUE(selected_bboxes[1]->region.rectified_roi == detect_bboxes[1]->region.rectified_roi);
}
}
}
}
