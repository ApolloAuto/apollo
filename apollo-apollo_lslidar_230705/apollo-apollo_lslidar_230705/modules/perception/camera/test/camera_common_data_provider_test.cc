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

#include "modules/perception/camera/common/data_provider.h"
#include "modules/perception/camera/test/camera_common_io_util.h"

namespace apollo {
namespace perception {

namespace common {
DECLARE_string(obs_sensor_meta_path);
DECLARE_string(obs_sensor_intrinsic_path);
}  // namespace common

namespace camera {

TEST(DataProvider, test_image_options) {
  unsetenv("MODULE_PATH");
  unsetenv("CYBER_PATH");
  DataProvider::ImageOptions image_options;
  EXPECT_EQ(image_options.ToString(), " 0 0");
  image_options.do_crop = true;
  EXPECT_EQ(image_options.ToString(), " 0 1 0 0 0 0");
}

TEST(DataProvider, test_nodistortion) {
  unsetenv("MODULE_PATH");
  unsetenv("CYBER_PATH");
  cv::Mat img = cv::imread(
      "/apollo/modules/perception/testdata/"
      "camera/common/img/test.jpg");

  DataProvider data_provider;
  DataProvider::InitOptions init_options;
  init_options.image_height = img.rows;
  init_options.image_width = img.cols;
  init_options.device_id = 0;
  data_provider.Init(init_options);

  base::Image8U image;
  DataProvider::ImageOptions image_options;

  EXPECT_FALSE(data_provider.GetImage(image_options, nullptr));

  image_options.target_color = base::Color::GRAY;
  EXPECT_FALSE(data_provider.GetImage(image_options, &image));
  image_options.target_color = base::Color::BGR;
  EXPECT_FALSE(data_provider.GetImage(image_options, &image));
  image_options.target_color = base::Color::RGB;
  EXPECT_FALSE(data_provider.GetImage(image_options, &image));

  EXPECT_TRUE(
      data_provider.FillImageData(img.rows, img.cols, img.data, "bgr8"));

  // test GetImage - correct color
  image_options.target_color = base::Color::GRAY;
  EXPECT_TRUE(data_provider.GetImage(image_options, &image));

  image_options.target_color = base::Color::BGR;
  EXPECT_TRUE(data_provider.GetImage(image_options, &image));

  image_options.target_color = base::Color::RGB;
  EXPECT_TRUE(data_provider.GetImage(image_options, &image));

  // test GetImage - wrong color
  image_options.target_color = base::Color::NONE;
  EXPECT_FALSE(data_provider.GetImage(image_options, &image));

  image_options.do_crop = true;
  image_options.crop_roi = base::RectI(100, 100, 512, 512);

  // test GetImage - with crop
  image_options.target_color = base::Color::GRAY;
  EXPECT_TRUE(data_provider.GetImage(image_options, &image));

  image_options.target_color = base::Color::BGR;
  EXPECT_TRUE(data_provider.GetImage(image_options, &image));

  image_options.target_color = base::Color::RGB;
  EXPECT_TRUE(data_provider.GetImage(image_options, &image));

  base::Blob<uint8_t> blob;

  // test GetImageBlob - correct color
  image_options.target_color = base::Color::GRAY;
  EXPECT_TRUE(data_provider.GetImageBlob(image_options, &blob));

  image_options.target_color = base::Color::BGR;
  EXPECT_TRUE(data_provider.GetImageBlob(image_options, &blob));

  image_options.target_color = base::Color::RGB;
  EXPECT_TRUE(data_provider.GetImageBlob(image_options, &blob));

  // test GetImageBlob - wrong color
  image_options.target_color = base::Color::NONE;
  EXPECT_FALSE(data_provider.GetImageBlob(image_options, &blob));
}

TEST(DataProvider, test_fill_image_data) {
  unsetenv("MODULE_PATH");
  unsetenv("CYBER_PATH");
  FLAGS_obs_sensor_meta_path =
      "/apollo/modules/perception/testdata/"
      "camera/common/conf/sensor_meta.config";
  FLAGS_obs_sensor_intrinsic_path =
      "/apollo/modules/perception/testdata/"
      "camera/common/params";

  cv::Mat gray;
  cv::Mat rgb;
  cv::Mat bgr = cv::imread(
      "/apollo/modules/perception/testdata/"
      "camera/common/img/test.jpg");
  cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

  DataProvider::InitOptions init_options;
  init_options.image_height = bgr.rows;
  init_options.image_width = bgr.cols;
  init_options.device_id = 0;

  // test fill without undistortion
  {
    DataProvider data_provider;
    EXPECT_TRUE(data_provider.Init(init_options));
    EXPECT_TRUE(
        data_provider.FillImageData(bgr.rows, bgr.cols, bgr.data, "bgr8"));
    EXPECT_TRUE(
        data_provider.FillImageData(rgb.rows, rgb.cols, rgb.data, "rgb8"));
    EXPECT_TRUE(
        data_provider.FillImageData(gray.rows, gray.cols, gray.data, "gray"));
    EXPECT_FALSE(
        data_provider.FillImageData(gray.rows, gray.cols, gray.data, "none"));
  }

  // test fill with undistortion
  init_options.do_undistortion = true;
  init_options.sensor_name = "onsemi_obstacle";
  {
    DataProvider data_provider;
    EXPECT_TRUE(data_provider.Init(init_options));
    EXPECT_TRUE(
        data_provider.FillImageData(bgr.rows, bgr.cols, bgr.data, "bgr8"));
    EXPECT_TRUE(
        data_provider.FillImageData(rgb.rows, rgb.cols, rgb.data, "rgb8"));
    EXPECT_TRUE(
        data_provider.FillImageData(gray.rows, gray.cols, gray.data, "gray"));
    EXPECT_FALSE(
        data_provider.FillImageData(gray.rows, gray.cols, gray.data, "none"));
  }
}

TEST(DataProvider, test_convert_color) {
  unsetenv("MODULE_PATH");
  unsetenv("CYBER_PATH");
  FLAGS_obs_sensor_meta_path =
      "/apollo/modules/perception/testdata/"
      "camera/common/conf/sensor_meta.config";
  FLAGS_obs_sensor_intrinsic_path =
      "/apollo/modules/perception/testdata/"
      "camera/common/params";

  cv::Mat gray;
  cv::Mat rgb;
  cv::Mat bgr = cv::imread(
      "/apollo/modules/perception/testdata/"
      "camera/common/img/test.jpg");
  cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

  DataProvider::InitOptions init_options;
  init_options.image_height = bgr.rows;
  init_options.image_width = bgr.cols;
  init_options.device_id = 0;

  base::Image8U image;

  // test get image without filling
  {
    DataProvider data_provider;
    EXPECT_TRUE(data_provider.Init(init_options));
    EXPECT_FALSE(data_provider.to_gray_image());
    EXPECT_FALSE(data_provider.to_bgr_image());
    EXPECT_FALSE(data_provider.to_rgb_image());
  }

  // fill bgr & to_bgr
  {
    DataProvider data_provider;
    EXPECT_TRUE(data_provider.Init(init_options));
    EXPECT_TRUE(
        data_provider.FillImageData(bgr.rows, bgr.cols, bgr.data, "bgr8"));
    EXPECT_TRUE(data_provider.to_bgr_image());
  }
  // fill bgr & to_rgb
  {
    DataProvider data_provider;
    EXPECT_TRUE(data_provider.Init(init_options));
    EXPECT_TRUE(
        data_provider.FillImageData(bgr.rows, bgr.cols, bgr.data, "bgr8"));
    EXPECT_TRUE(data_provider.to_rgb_image());
  }
  // fill bgr & to_gray
  {
    DataProvider data_provider;
    EXPECT_TRUE(data_provider.Init(init_options));
    EXPECT_TRUE(
        data_provider.FillImageData(bgr.rows, bgr.cols, bgr.data, "bgr8"));
    EXPECT_TRUE(data_provider.to_gray_image());
  }

  // fill rgb & to_bgr
  {
    DataProvider data_provider;
    EXPECT_TRUE(data_provider.Init(init_options));
    EXPECT_TRUE(
        data_provider.FillImageData(rgb.rows, rgb.cols, rgb.data, "rgb8"));
    EXPECT_TRUE(data_provider.to_bgr_image());
  }
  // fill rgb & to_rgb
  {
    DataProvider data_provider;
    EXPECT_TRUE(data_provider.Init(init_options));
    EXPECT_TRUE(
        data_provider.FillImageData(rgb.rows, rgb.cols, rgb.data, "rgb8"));
    EXPECT_TRUE(data_provider.to_rgb_image());
  }
  // fill rgb & to_gray
  {
    DataProvider data_provider;
    EXPECT_TRUE(data_provider.Init(init_options));
    EXPECT_TRUE(
        data_provider.FillImageData(rgb.rows, rgb.cols, rgb.data, "rgb8"));
    EXPECT_TRUE(data_provider.to_gray_image());
  }

  // fill gray & to_bgr
  {
    DataProvider data_provider;
    EXPECT_TRUE(data_provider.Init(init_options));
    EXPECT_TRUE(
        data_provider.FillImageData(gray.rows, gray.cols, gray.data, "gray"));
    EXPECT_TRUE(data_provider.to_bgr_image());
  }
  // fill gray & to_rgb
  {
    DataProvider data_provider;
    EXPECT_TRUE(data_provider.Init(init_options));
    EXPECT_TRUE(
        data_provider.FillImageData(gray.rows, gray.cols, gray.data, "gray"));
    EXPECT_TRUE(data_provider.to_rgb_image());
  }
  // fill gray & to_gray
  {
    DataProvider data_provider;
    EXPECT_TRUE(data_provider.Init(init_options));
    EXPECT_TRUE(
        data_provider.FillImageData(gray.rows, gray.cols, gray.data, "gray"));
    EXPECT_TRUE(data_provider.to_gray_image());
  }
}

TEST(DataProvider, test_undistortion) {
  unsetenv("MODULE_PATH");
  unsetenv("CYBER_PATH");
  FLAGS_obs_sensor_meta_path =
      "/apollo/modules/perception/testdata/"
      "camera/common/conf/sensor_meta.config";
  FLAGS_obs_sensor_intrinsic_path =
      "/apollo/modules/perception/testdata/"
      "camera/common/params";

  cv::Mat img = cv::imread(
      "/apollo/modules/perception/testdata/"
      "camera/common/img/test.jpg");

  DataProvider data_provider;
  DataProvider::InitOptions init_options;
  init_options.image_height = img.rows;
  init_options.image_width = img.cols;
  init_options.do_undistortion = true;
  init_options.sensor_name = "none_onsemi_obstacle";
  init_options.device_id = 0;
  EXPECT_FALSE(data_provider.Init(init_options));
  init_options.sensor_name = "onsemi_obstacle";
  EXPECT_TRUE(data_provider.Init(init_options));

  EXPECT_TRUE(
      data_provider.FillImageData(img.rows, img.cols, img.data, "bgr8"));

  base::Image8U image;
  DataProvider::ImageOptions image_options;

  image_options.target_color = base::Color::GRAY;
  EXPECT_TRUE(data_provider.GetImage(image_options, &image));

  image_options.target_color = base::Color::BGR;
  EXPECT_TRUE(data_provider.GetImage(image_options, &image));

  image_options.target_color = base::Color::RGB;
  EXPECT_TRUE(data_provider.GetImage(image_options, &image));

  image_options.do_crop = true;
  image_options.crop_roi = base::RectI(100, 100, 512, 512);

  image_options.target_color = base::Color::GRAY;
  EXPECT_TRUE(data_provider.GetImage(image_options, &image));

  image_options.target_color = base::Color::BGR;
  EXPECT_TRUE(data_provider.GetImage(image_options, &image));

  image_options.target_color = base::Color::RGB;
  EXPECT_TRUE(data_provider.GetImage(image_options, &image));

  base::Blob<uint8_t> blob;

  image_options.target_color = base::Color::GRAY;
  EXPECT_TRUE(data_provider.GetImageBlob(image_options, &blob));

  image_options.target_color = base::Color::BGR;
  EXPECT_TRUE(data_provider.GetImageBlob(image_options, &blob));
  // get the same image twice
  EXPECT_TRUE(data_provider.GetImageBlob(image_options, &blob));

  image_options.target_color = base::Color::RGB;
  EXPECT_TRUE(data_provider.GetImageBlob(image_options, &blob));
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
