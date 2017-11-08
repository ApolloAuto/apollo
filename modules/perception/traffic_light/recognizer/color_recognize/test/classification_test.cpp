//
// Created by gaohan02 on 16-12-1.
//

#include "gtest/gtest.h"
#include "module/perception/traffic_light/recognizer/color_recognize/color_recognize.h"

namespace adu {
namespace perception {
namespace traffic_light {

class ClassificationTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    _recognizer = new ColorRecognize;
    ASSERT_TRUE(_recognizer->init());
    AINFO << "Setup";
  }
  ~ClassificationTest() {
    delete _recognizer;
  }
 protected:
  BaseRecognizer *_recognizer;
};

TEST_F(ClassificationTest, red) {
  using cv::Rect;
  cv::Mat red_img;
  red_img = cv::imread("./img/red.jpg");
  ASSERT_FALSE(red_img.data == NULL) << "load img failed";

  Image image;
  ASSERT_TRUE(image.init(0.0, LONG_FOCUS, red_img)) << "init image failed.";

  std::vector<LightPtr> lights;
  lights.emplace_back(new Light);
  cv::Rect region(1043, 601, 21, 54);
  lights[0]->region.is_detected = true;
  lights[0]->region.rectified_roi = region;

#ifndef CPU_ONLY
  ASSERT_TRUE(_recognizer->recognize_status(image, RecognizeOption(),
                                            &lights));

  ASSERT_EQ(lights[0]->status.color, RED);
#endif
}

TEST_F(ClassificationTest, green) {
  using cv::Rect;
  cv::Mat green_img;
  green_img = cv::imread("./img/green.jpg");
  ASSERT_FALSE(green_img.data == NULL) << "load img failed";

  Image image;
  ASSERT_TRUE(image.init(0.0, LONG_FOCUS, green_img)) << "init image failed.";

  std::vector<LightPtr> lights;
  lights.emplace_back(new Light);
  cv::Rect region(538, 332, 52, 135);
  lights[0]->region.is_detected = true;
  lights[0]->region.rectified_roi = region;

#ifndef CPU_ONLY
  ASSERT_TRUE(_recognizer->recognize_status(image, RecognizeOption(),
                                            &lights));

  ASSERT_EQ(lights[0]->status.color, GREEN);
#endif
}

}
}
}
