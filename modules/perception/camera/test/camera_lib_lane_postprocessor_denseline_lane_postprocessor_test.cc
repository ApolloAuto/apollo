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
#include <opencv2/opencv.hpp>
#include "gtest/gtest.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/base/distortion_model.h"
#include "modules/perception/camera/common/camera_frame.h"
#include "modules/perception/camera/lib/calibration_service/online_calibration_service/online_calibration_service.h"  // NOLINT
#include "modules/perception/camera/lib/lane/detector/denseline/denseline_lane_detector.h"
#include "modules/perception/camera/lib/lane/postprocessor/denseline/denseline_lane_postprocessor.h"
#include "modules/perception/common/io/io_util.h"

namespace apollo {
namespace perception {
namespace camera {

TEST(DenselineLanePostprocessor, camera_lane_postprocessor_point_test) {
  unsetenv("MODULE_PATH");
  unsetenv("CYBER_PATH");
  //  initialize lane detector
  LaneDetectorInitOptions init_options;
  LaneDetectorOptions detetor_options;
  init_options.conf_file = "config.pt";
  init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/lib/lane/postprocessor/denseline/data/";
  base::BrownCameraDistortionModel model;
  common::LoadBrownCameraIntrinsic(
      "/apollo/modules/perception/testdata/"
      "camera/lib/lane/postprocessor/denseline/params/"
      "onsemi_obstacle_intrinsics.yaml",
      &model);
  init_options.base_camera_model = model.get_camera_model();
  //  BaseLaneDetector *detector =
  //      BaseLaneDetectorRegisterer::GetInstanceByName("DenselineLaneDetector");
  std::shared_ptr<DenselineLaneDetector> detector(new DenselineLaneDetector);
  AINFO << "detector: " << detector->Name();
  EXPECT_TRUE(detector->Init(init_options));
  //  initialize lane postprocessor
  std::shared_ptr<DenselineLanePostprocessor> lane_postprocessor;
  lane_postprocessor.reset(new DenselineLanePostprocessor);
  LanePostprocessorInitOptions postprocessor_init_options;
  postprocessor_init_options.detect_config_root =
      "/apollo/modules/perception/testdata/"
      "camera/lib/lane/postprocessor/denseline/data/";
  postprocessor_init_options.detect_config_name = "config.pt";
  postprocessor_init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/lib/lane/postprocessor/denseline/data/";
  postprocessor_init_options.conf_file = "lane_postprocessor_config.pt";
  lane_postprocessor->Init(postprocessor_init_options);
  LanePostprocessorOptions postprocessor_options;
  const std::string result_dir = "./result";
  cyber::common::EnsureDirectory(result_dir);

  //  set pitch angle
  float pitch_angle = 0.0f;
  //  set camera_ground_height (unit:meter)
  float camera_ground_height = 1.6f;
  CameraFrame frame;
  frame.camera_k_matrix = model.get_intrinsic_params();
  CalibrationServiceInitOptions calibration_service_init_options;
  calibration_service_init_options.calibrator_working_sensor_name =
      "onsemi_obstacle";
  std::map<std::string, Eigen::Matrix3f> name_intrinsic_map;
  name_intrinsic_map["onsemi_obstacle"] = frame.camera_k_matrix;
  calibration_service_init_options.name_intrinsic_map = name_intrinsic_map;
  calibration_service_init_options.calibrator_method = "LaneLineCalibrator";
  calibration_service_init_options.image_height =
      static_cast<int>(model.get_height());
  calibration_service_init_options.image_width =
      static_cast<int>(model.get_width());
  OnlineCalibrationService calibration_service;
  ACHECK(calibration_service.Init(calibration_service_init_options));

  std::map<std::string, float> name_camera_ground_height_map;
  std::map<std::string, float> name_camera_pitch_angle_diff_map;
  name_camera_ground_height_map["onsemi_obstacle"] = camera_ground_height;
  name_camera_pitch_angle_diff_map["onsemi_obstacle"] = 0;
  calibration_service.SetCameraHeightAndPitch(name_camera_ground_height_map,
                                              name_camera_pitch_angle_diff_map,
                                              pitch_angle);
  frame.calibration_service = &calibration_service;

  //  image data initialized
  std::shared_ptr<base::SyncedMemory> img_gpu_data;
  DataProvider::InitOptions dp_init_options;
  DataProvider data_provider;
  frame.data_provider = &data_provider;

  for (int i = 1; i < 5; i++) {
    std::string impath = cv::format(
        "/apollo/modules/perception/testdata/"
        "camera/lib/lane/postprocessor/"
        "denseline/data/test_%d.jpg",
        i);
    cv::Mat img = cv::imread(impath);
    ACHECK(!img.empty()) << "input image is empty.";
    int size = img.cols * img.rows * img.channels();
    img_gpu_data.reset(new base::SyncedMemory(size, true));
    memcpy(img_gpu_data->mutable_cpu_data(), img.data, size * sizeof(uint8_t));
    dp_init_options.image_height = img.rows;
    dp_init_options.image_width = img.cols;
    dp_init_options.device_id = 0;
    frame.data_provider->Init(dp_init_options);
    frame.data_provider->FillImageData(
        img.rows, img.cols, (const uint8_t *)(img_gpu_data->mutable_gpu_data()),
        "bgr8");
    EXPECT_TRUE(detector->Detect(detetor_options, &frame));
    AINFO << "detector finished!";
    //  postprocess
    EXPECT_EQ(lane_postprocessor->Name(), "DenselineLanePostprocessor");
    lane_postprocessor->Process2D(postprocessor_options, &frame);
    lane_postprocessor->Process3D(postprocessor_options, &frame);

    std::vector<std::vector<LanePointInfo> > laneline_point_set =
        lane_postprocessor->GetLanelinePointSet();
    std::vector<LanePointInfo> laneline_infer_set =
        lane_postprocessor->GetAllInferLinePointSet();
    std::vector<ConnectedComponent> ccs;
    std::vector<ConnectedComponent> select_ccs;
    std::vector<unsigned char> lane_map;
    int lane_map_width = 0;
    int lane_map_height = 0;
    lane_postprocessor->GetLaneCCs(&lane_map, &lane_map_width, &lane_map_height,
                                   &ccs, &select_ccs);
    // delete detector;
  }
}

TEST(DenselineLanePostprocessor, lane_postprocessor_init_test) {
  unsetenv("MODULE_PATH");
  unsetenv("CYBER_PATH");
  std::shared_ptr<DenselineLanePostprocessor> lane_postprocessor;
  lane_postprocessor.reset(new DenselineLanePostprocessor);
  LanePostprocessorInitOptions postprocessor_init_options;

  postprocessor_init_options.detect_config_root =
      "/apollo/modules/perception/testdata/"
      "camera/lib/lane/postprocessor/denseline/data/";
  postprocessor_init_options.detect_config_name = "error_config.pt";
  postprocessor_init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/lib/lane/postprocessor/denseline/data/";
  postprocessor_init_options.conf_file = "lane_postprocessor_config.pt";
  EXPECT_FALSE(lane_postprocessor->Init(postprocessor_init_options));

  postprocessor_init_options.detect_config_root =
      "/apollo/modules/perception/testdata/"
      "camera/lib/lane/postprocessor/denseline/data/";
  postprocessor_init_options.detect_config_name = "config.pt";
  postprocessor_init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/lib/lane/postprocessor/denseline/data/";
  postprocessor_init_options.conf_file = "error_lane_postprocessor_config.pt";
  EXPECT_FALSE(lane_postprocessor->Init(postprocessor_init_options));
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
