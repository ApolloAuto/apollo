/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <opencv2/opencv.hpp>
#include "gtest/gtest.h"

#include "cyber/common/log.h"
#include "modules/perception/base/distortion_model.h"
#include "modules/perception/camera/lib/lane/detector/darkSCNN/darkSCNN_lane_detector.h"
#include "modules/perception/common/io/io_util.h"

namespace apollo {
namespace perception {
namespace camera {

TEST(darkSCNNLaneDetector, darkSCNN_lane_detector_test) {
  LaneDetectorInitOptions init_options;
  LaneDetectorOptions detetor_options;
  CameraFrame frame;
  cv::Mat img = cv::imread(
      "/apollo/modules/perception/testdata/"
      "camera/lib/lane/detector/darkSCNN/data/test.jpg");
  CHECK(!img.empty()) << "input image is empty.";

  std::cout << img.rows << std::endl;

  std::shared_ptr<base::SyncedMemory> img_gpu_data;
  int size = img.cols * img.rows * img.channels();
  img_gpu_data.reset(new base::SyncedMemory(size, true));
  memcpy(img_gpu_data->mutable_cpu_data(), img.data, size * sizeof(uint8_t));

  DataProvider data_provider;
  frame.data_provider = &data_provider;
  DataProvider::InitOptions dp_init_options;
  dp_init_options.image_height = img.rows;
  dp_init_options.image_width = img.cols;
  dp_init_options.device_id = 0;
  EXPECT_TRUE(frame.data_provider->Init(dp_init_options));
  frame.data_provider->FillImageData(
      img.rows, img.cols, (const uint8_t *)(img_gpu_data->mutable_gpu_data()),
      "bgr8");

  std::shared_ptr<DarkSCNNLaneDetector> detector(new DarkSCNNLaneDetector);
  AINFO << "detector: " << detector->Name();

  // init_options.conf_file = "";
  // init_options.root_dir = "";
  // EXPECT_FALSE(detector->Init(init_options));

  init_options.conf_file = "config_for_test.pt";
  init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/lib/lane/detector/darkSCNN/data/";
  EXPECT_TRUE(detector->Init(init_options));

  init_options.conf_file = "config.pt";
  init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/lib/lane/detector/darkSCNN/data/";
  EXPECT_TRUE(detector->Init(init_options));

  AINFO << "init";

  base::BrownCameraDistortionModel model;
  common::LoadBrownCameraIntrinsic(
      "/apollo/modules/perception/testdata/"
      "camera/lib/lane/detector/darkSCNN/params/"
      "onsemi_obstacle_intrinsics.yaml",
      &model);
  init_options.base_camera_model = model.get_camera_model();
  EXPECT_TRUE(detector->Init(init_options));
  EXPECT_TRUE(detector->Detect(detetor_options, &frame));
  AINFO << "detector finished!";

  // delete detector;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
