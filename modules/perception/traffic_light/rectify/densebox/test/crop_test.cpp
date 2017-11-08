//
// Created by gaohan02 on 16-9-14.
//

#include <gtest/gtest.h>
#include "module/perception/traffic_light/rectify/densebox/crop/cropbox.h"

namespace adu {
namespace perception {
namespace traffic_light {

class CropBoxTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    _crop_scale = 3;
    _min_crop_size = 270;
    _crop_local = new CropBox(_crop_scale, _min_crop_size);
  }
  ~CropBoxTest() {
    delete (_crop_local);
  }

 protected:
  IGetBox *_crop_local;
  float _crop_scale;
  int _min_crop_size;
};

TEST_F(CropBoxTest, crop0) {
  std::vector<BoundBox_t> hd_box(1);
  hd_box[0].rect = cv::Rect(1898, 798, 21, 177);
  hd_box[0].isValid = false;
  cv::Mat blank(1080, 1920, CV_8U);
  BoundBox_t cbox;
  _crop_local->get_crop_box(blank, hd_box, cbox);
  cv::Rect rect(0, 0, 0, 0);
  ASSERT_TRUE(rect == cbox.rect);
}

TEST_F(CropBoxTest, crop1) {
  std::vector<BoundBox_t> hd_box(1);
  hd_box[0].rect = cv::Rect(1898, 798, 21, 177);
  hd_box[0].isValid = true;
  cv::Mat blank(1080, 1920, CV_8U);
  BoundBox_t cbox;
  _crop_local->get_crop_box(blank, hd_box, cbox);
  //cv::Rect rect(1649, 621, 270, 458);
  cv::Rect rect(1877, 621, 42, 458);
  ASSERT_TRUE(rect == cbox.rect) << rect << cbox.rect;

  hd_box[0].rect = cv::Rect(1811, 803, 21, 177);
  _crop_local->get_crop_box(blank, hd_box, cbox);
  //rect = cv::Rect(1649, 626, 270, 453);
  rect = cv::Rect(1790, 626, 63, 453);
  ASSERT_TRUE(rect == cbox.rect) << rect << cbox.rect;
}

TEST_F(CropBoxTest, crop2) {
  std::vector<BoundBox_t> hd_box(2);
  hd_box[0].rect = cv::Rect(1439, 743, 30, 192);
  hd_box[0].isValid = true;
  hd_box[1].rect = cv::Rect(1620, 743, 27, 192);
  hd_box[1].isValid = true;
  cv::Mat blank(1080, 1920, CV_8U);
  BoundBox_t cbox;
  _crop_local->get_crop_box(blank, hd_box, cbox);
  cv::Rect rect(1231, 551, 624, 528);
  ASSERT_TRUE(rect == cbox.rect);
}

}
}
}