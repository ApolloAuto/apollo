#include <gtest/gtest.h>
#include <sstream>

#include "modules/perception/traffic_light/base/image.h"

namespace adu {
namespace perception {
namespace traffic_light {

class ImageTest : public ::testing::Test {
 public:
  ImageTest() {}
  virtual ~ImageTest() {}
 protected:
  virtual void SetUp() override {
    _image = new Image();
  }

  virtual void TearDown() override {
    delete _image;
  }

 protected:
  Image *_image;
};

TEST_F(ImageTest, test_all) {
  cv::Mat img;
  CameraId cam_id = CameraId::LONG_FOCUS;
  double timestamp = 0.0;
  {
    std::stringstream ss;
    ss << *_image;
    EXPECT_EQ("Image not inited.", ss.str());
  }

  ASSERT_TRUE(_image->init(timestamp, cam_id, img));
  {
    std::stringstream ss;
    ss << *_image;
    ASSERT_FALSE("Image not inited." == ss.str());
  }

  EXPECT_EQ(CameraId::LONG_FOCUS, _image->device_id());
  EXPECT_DOUBLE_EQ(0.0, _image->ts());
  EXPECT_EQ("long_focus_camera(25mm)", _image->device_id_str());

  cam_id = CameraId::UNKNOWN;
  ASSERT_TRUE(_image->init(timestamp, cam_id, img));
  EXPECT_EQ("unkown device(camera)", _image->device_id_str());
}

}
}
}
