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

#include "modules/perception/camera/common/undistortion_handler.h"
#include "modules/perception/camera/test/camera_common_io_util.h"
#include "modules/perception/camera/test/camera_common_undistortion.h"
#include "modules/perception/lib/utils/timer.h"

namespace apollo {
namespace perception {

namespace common {
DECLARE_string(obs_sensor_meta_path);
DECLARE_string(obs_sensor_intrinsic_path);
}  // namespace common

namespace camera {

TEST(UndistortionHandlerTest, test_init) {
  unsetenv("MODULE_PATH");
  unsetenv("CYBER_PATH");
  {
    FLAGS_obs_sensor_meta_path =
        "/apollo/modules/perception/testdata/"
        "camera/common/conf/sensor_meta.config";
    FLAGS_obs_sensor_intrinsic_path =
        "/apollo/modules/perception/testdata/"
        "camera/common/params";
    cv::Mat cv_img = cv::imread(
        "/apollo/modules/perception/testdata/"
        "camera/common/img/origin.jpeg");

    base::Image8U image(cv_img.rows, cv_img.cols, base::Color::BGR);
    base::Image8U undistorted(cv_img.rows, cv_img.cols, base::Color::BGR);

    for (int y = 0; y < cv_img.rows; ++y) {
      memcpy(image.mutable_cpu_ptr(y), cv_img.ptr<uint8_t>(y),
             image.width_step());
    }
    UndistortionHandler handler;

    // undistortion before init
    EXPECT_FALSE(handler.Handle(image, &undistorted));

    // init undistortion
    EXPECT_FALSE(handler.Init("none_onsemi_obstacle", 0));
    EXPECT_FALSE(handler.Init("onsemi_obstacle", -1));
    EXPECT_TRUE(handler.Init("onsemi_obstacle", 0));
    EXPECT_TRUE(handler.Init("onsemi_obstacle", 0));

    EXPECT_TRUE(handler.Handle(image, &undistorted));
  }
}

TEST(UndistortionHandlerTest, test_undistortion) {
  unsetenv("MODULE_PATH");
  unsetenv("CYBER_PATH");
  {
    FLAGS_obs_sensor_meta_path =
        "/apollo/modules/perception/testdata/"
        "camera/common/conf/sensor_meta.config";
    FLAGS_obs_sensor_intrinsic_path =
        "/apollo/modules/perception/testdata/"
        "camera/common/params";
    cv::Mat cv_bgr = cv::imread(
        "/apollo/modules/perception/testdata/"
        "camera/common/img/origin.jpeg");
    cv::Mat cv_gray = cv::imread(
        "/apollo/modules/perception/testdata/"
        "camera/common/img/origin.jpeg",
        CV_LOAD_IMAGE_GRAYSCALE);

    base::Image8U bgr(cv_bgr.rows, cv_bgr.cols, base::Color::BGR);
    base::Image8U bgr_undistorted(cv_bgr.rows, cv_bgr.cols, base::Color::BGR);

    base::Image8U gray(cv_bgr.rows, cv_bgr.cols, base::Color::GRAY);
    base::Image8U gray_undistorted(cv_bgr.rows, cv_bgr.cols, base::Color::GRAY);

    for (int y = 0; y < cv_bgr.rows; ++y) {
      memcpy(bgr.mutable_cpu_ptr(y), cv_bgr.ptr<uint8_t>(y), bgr.width_step());
    }

    for (int y = 0; y < cv_gray.rows; ++y) {
      memcpy(gray.mutable_cpu_ptr(y), cv_gray.ptr<uint8_t>(y),
             gray.width_step());
    }

    UndistortionHandler handler;

    // init undistortion
    EXPECT_TRUE(handler.Init("onsemi_obstacle", 0));

    EXPECT_TRUE(handler.Handle(bgr, &bgr_undistorted));
    EXPECT_TRUE(handler.Handle(gray, &gray_undistorted));
  }
}

TEST(UndistortionHandlerTest, test_verify) {
  unsetenv("MODULE_PATH");
  unsetenv("CYBER_PATH");
  {
    // test RGB undistortion
    cv::Mat rgb_image = cv::imread(
        "/apollo/modules/perception/testdata/"
        "camera/common/img/origin.jpeg");
    int img_width = rgb_image.cols;
    int img_height = rgb_image.rows;

    size_t image_len = img_height * img_width;
    size_t rgb_len = image_len * 3;
    uint8_t* rgb = new uint8_t[rgb_len];
    int err = -1;

    int num_repeats = 1000;

    std::vector<uint8_t> rgb_orig;

    rgb_orig.resize(image_len * 3);
    for (int r = 0; r < img_height; ++r) {
      for (int c = 0; c < img_width; ++c) {
        for (int ch = 0; ch < 3; ++ch) {
          rgb_orig[r * img_width * 3 + c * 3 + ch] =
              rgb_image.at<cv::Vec3b>(r, c)[ch];
        }
      }
    }

    ImageGpuPreprocessHandler handler;
    lib::Timer timer;
    timer.Start();
    err = handler.init(
        "/apollo/modules/perception/testdata/"
        "camera/common/params/onsemi_obstacle_intrinsics.yaml",
        0);
    AINFO << "Time cost rgb with undistort init : " << timer.End("") << " us";

    EXPECT_EQ(err, 0);

    // warm up
    err = handler.handle(rgb_orig.data(), rgb);
    EXPECT_EQ(err, 0);

    timer.Start();
    for (int i = 0; i < num_repeats; ++i) {
      err += handler.handle(rgb_orig.data(), rgb);
    }
    AINFO << "Time cost rgb with undistort handle : "
          << (timer.End("") / num_repeats) << " us";

    EXPECT_EQ(err, 0);

    {
      cv::Mat in_image(img_height, img_width, CV_8UC3, rgb);
      cv::imwrite("undistorted_old.jpg", in_image);
    }

    delete[] rgb;
  }

  {
    FLAGS_obs_sensor_meta_path =
        "/apollo/modules/perception/testdata/"
        "camera/common/conf/sensor_meta.config";
    FLAGS_obs_sensor_intrinsic_path =
        "/apollo/modules/perception/testdata/"
        "camera/common/params";
    cv::Mat cv_img = cv::imread(
        "/apollo/modules/perception/testdata/"
        "camera/common/img/origin.jpeg");

    base::Image8U image(cv_img.rows, cv_img.cols, base::Color::BGR);
    base::Image8U undistorted(cv_img.rows, cv_img.cols, base::Color::BGR);

    for (int y = 0; y < cv_img.rows; ++y) {
      memcpy(image.mutable_cpu_ptr(y), cv_img.ptr<uint8_t>(y),
             image.width_step());
    }
    UndistortionHandler handler;
    lib::Timer timer;
    timer.Start();
    EXPECT_TRUE(handler.Init("onsemi_obstacle", 0));
    timer.End("init UndistortionHandler");
    handler.Handle(image, &undistorted);
  }
  {
    cv::Mat img_old = cv::imread("undistorted_old.jpg");
    cv::Mat img_new = cv::imread("undistorted_new.jpg");
    for (int y = 0; y < img_old.rows; ++y) {
      auto old_ptr = img_old.ptr<cv::Vec3b>(y);
      auto new_ptr = img_new.ptr<cv::Vec3b>(y);
      for (int x = 0; x < img_old.cols; ++x) {
        EXPECT_NEAR(old_ptr[x][0], new_ptr[x][0], 5);
        EXPECT_NEAR(old_ptr[x][1], new_ptr[x][1], 5);
        EXPECT_NEAR(old_ptr[x][2], new_ptr[x][2], 5);
      }
    }
  }
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
