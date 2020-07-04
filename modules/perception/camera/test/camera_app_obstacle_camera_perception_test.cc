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

#include "modules/perception/camera/app/obstacle_camera_perception.h"

DEFINE_int32(height, 1080, "image height");
DEFINE_int32(width, 1920, "image width");

namespace apollo {
namespace perception {
namespace common {
DECLARE_string(obs_sensor_meta_path);
DECLARE_string(obs_sensor_intrinsic_path);
}  // namespace common
namespace camera {
TEST(ObstacleCameraPerceptionTest, init_all_test) {
  unsetenv("MODULE_PATH");
  unsetenv("CYBER_PATH");

  FLAGS_obs_sensor_intrinsic_path =
      "/apollo/modules/perception/testdata/"
      "camera/app/data/perception/camera/params";
  FLAGS_obs_sensor_meta_path =
      "/apollo/modules/perception/testdata/"
      "camera/app/data/perception/camera/sensor_meta_camera.pt";
  // test conf file
  {
    ObstacleCameraPerception perception;
    CameraPerceptionInitOptions init_option;
    init_option.root_dir =
        "/apollo/modules/perception/testdata/"
        "camera/app/conf/perception/camera/obstacle";
    init_option.conf_file = "obstacle.pt";
    ASSERT_TRUE(perception.Init(init_option));
  }
}

TEST(ObstacleCameraPerceptionTest, init_feature_extractor_test) {
  unsetenv("MODULE_PATH");
  unsetenv("CYBER_PATH");
  FLAGS_obs_sensor_meta_path =
      "/apollo/modules/perception/testdata/"
      "camera/app/data/perception/camera/sensor_meta_camera.pt";
  ObstacleCameraPerception perception;
  CameraPerceptionInitOptions init_option;
  init_option.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/app/conf/perception/camera/obstacle";
  init_option.conf_file = "obstacle_no_feature_extractor.pt";
  ASSERT_TRUE(perception.Init(init_option));
}

TEST(ObstacleCameraPerceptionTest, init_debug_para_test) {
  unsetenv("MODULE_PATH");
  unsetenv("CYBER_PATH");
  FLAGS_obs_sensor_meta_path =
      "/apollo/modules/perception/testdata/"
      "camera/app/data/perception/camera/sensor_meta_camera.pt";
  ObstacleCameraPerception perception;
  CameraPerceptionInitOptions init_option;
  init_option.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/app/conf/perception/camera/obstacle";
  init_option.conf_file = "obstacle_no_debug_para.pt";
  ASSERT_TRUE(perception.Init(init_option));
}

TEST(ObstacleCameraPerceptionTest, perception_test) {
  unsetenv("MODULE_PATH");
  unsetenv("CYBER_PATH");
  FLAGS_obs_sensor_meta_path =
      "/apollo/modules/perception/testdata/"
      "camera/app/data/perception/camera/sensor_meta_camera.pt";
  ObstacleCameraPerception perception;

  CameraPerceptionInitOptions init_option;
  CameraPerceptionOptions options;
  init_option.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/app/conf/perception/camera/obstacle";
  init_option.conf_file = "obstacle.pt";
  init_option.lane_calibration_working_sensor_name = "front_6mm";
  ASSERT_TRUE(perception.Init(init_option));

  const int FRAME_CAPACITY = 20;
  std::vector<CameraFrame> frame_list(FRAME_CAPACITY);
  DataProvider::InitOptions data_options;
  data_options.image_height = FLAGS_height;
  data_options.image_width = FLAGS_width;
  data_options.do_undistortion = false;
  data_options.device_id = 0;
  data_options.sensor_name = "front_6mm";
  DataProvider data_provider;
  ACHECK(data_provider.Init(data_options));
  for (auto &frame : frame_list) {
    frame.track_feature_blob.reset(new base::Blob<float>());
    frame.data_provider = &data_provider;
  }
  int frame_id = -1;

  std::ifstream fin;
  fin.open(
      "/apollo/modules/perception/testdata/"
      "camera/app/images/test.txt",
      std::ifstream::in);
  ASSERT_TRUE(fin.is_open());
  std::string image_name;
  std::map<std::string, float> name_camera_ground_height_map;
  std::map<std::string, float> name_camera_pitch_angle_diff_map;
  name_camera_ground_height_map["front_6mm"] = 1.6f;
  name_camera_pitch_angle_diff_map["front_6mm"] = 0.0;
  float pitch_angle = 0.f;
  perception.SetCameraHeightAndPitch(name_camera_ground_height_map,
                                     name_camera_pitch_angle_diff_map,
                                     pitch_angle);
  while (fin >> image_name) {
    AINFO << "image: " << image_name;
    std::string image_path =
        "/apollo/modules/perception/testdata/"
        "camera/app/images/" +
        image_name + ".jpg";
    cv::Mat image;
    image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);
    ASSERT_TRUE(image.data);
    frame_id++;
    CameraFrame &frame = frame_list[frame_id % FRAME_CAPACITY];
    frame.camera2world_pose *= Eigen::Scaling(0.0);
    frame.camera2world_pose(0, 2) = 1.0f;
    frame.camera2world_pose(1, 0) = -1.0f;
    frame.camera2world_pose(2, 1) = -1.0f;
    frame.data_provider->FillImageData(image.rows, image.cols,
                                       (const uint8_t *)(image.data), "bgr8");
    frame.frame_id = frame_id;
    frame.timestamp = frame_id * 0.066;
    perception.GetCalibrationService(&frame.calibration_service);
    EXPECT_TRUE(perception.Perception(options, &frame));
  }
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
