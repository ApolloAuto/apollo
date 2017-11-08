#include <gtest/gtest.h>

#include "module/perception/traffic_light/base/utils.h"

namespace adu {
namespace perception {
namespace traffic_light {

TEST(UtilsTest, test_refined_box) {
  cv::Size size(200, 200);
  cv::Rect rect1(-1, -1, 200, 200);
  auto box1 = refined_box(rect1, size);
  EXPECT_EQ(0, box1.x);
  EXPECT_EQ(0, box1.y);
  EXPECT_EQ(199, box1.width);
  EXPECT_EQ(199, box1.height);

  cv::Rect rect2(-300, -300, -10, -10);
  auto box2 = refined_box(rect2, size);
  EXPECT_EQ(0, box2.x);
  EXPECT_EQ(0, box2.y);
  EXPECT_EQ(0, box2.width);
  EXPECT_EQ(0, box2.height);

  cv::Rect rect3(10, 10, 300, 300);
  auto box3 = refined_box(rect3, size);
  EXPECT_EQ(10, box3.x);
  EXPECT_EQ(10, box3.y);
  EXPECT_EQ(189, box3.width);
  EXPECT_EQ(189, box3.height);
}

}
}
}
