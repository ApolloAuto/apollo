//
// Created by gaohan02 on 16-12-1.
//

#include "gtest/gtest.h"
#include "module/perception/traffic_light/rectify/densebox/detection/detection.h"
#include "module/perception/traffic_light/rectify/densebox/densebox_rectify.h"

namespace adu {
namespace perception {
namespace traffic_light {

class SynthesisTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    _rectifier = new DenseBoxRectify;
    ASSERT_TRUE(_rectifier->init());
    _origin_image = cv::imread("./img/img.png");
    AINFO << "Setup";
  }
  ~SynthesisTest() {
    delete _rectifier;
  }
 protected:

  cv::Mat _origin_image;
  BaseRectifier *_rectifier;
};

TEST_F(SynthesisTest, all) {
  using cv::Rect;
  ASSERT_FALSE(_origin_image.data == NULL) << "load img failed";

  Image image;
  ASSERT_TRUE(image.init(0.0, LONG_FOCUS, _origin_image)) << "init image failed.";

  std::vector<LightPtr> lights;
  lights.emplace_back(new Light);
  lights.emplace_back(new Light);

  lights[0]->region.projection_roi.x = 1150;
  lights[0]->region.projection_roi.y = 400;
  lights[0]->region.projection_roi.width = 50;
  lights[0]->region.projection_roi.height = 130;

  lights[1]->region.projection_roi.x = 1500;
  lights[1]->region.projection_roi.y = 390;
  lights[1]->region.projection_roi.width = 50;
  lights[1]->region.projection_roi.height = 130;

  std::vector<Rect> gt_lights;
  Rect light1(1193, 419, 45, 129);
  Rect light2(1551, 420, 45, 129);
  gt_lights.push_back(light1);
  gt_lights.push_back(light2);

#ifndef CPU_ONLY
  ASSERT_TRUE(_rectifier->rectify(image, RectifyOption(), &lights)) << "rectifier image failed.";

  ASSERT_EQ(lights.size(), gt_lights.size());
  for (int i = 0; i < lights.size(); i++) {
    bool get = false;
    Rect region = lights[i]->region.rectified_roi;
    AINFO << region;
    for (int j = 0; j < gt_lights.size(); j++) {
      cv::Rect _inter = region & gt_lights[j];
      cv::Rect _union = region | gt_lights[j];
      float iou = float(_inter.area()) / float(_union.area() + 1e-6);
      AINFO << "IOU " << iou;
      if (iou > 0.5) {
        get = true;
        break;
      }
    }
    ASSERT_TRUE(get);
  }
#endif
}
}
}
}
