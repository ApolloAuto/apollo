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

#include "modules/perception/camera/app/traffic_light_camera_perception.h"
#include "modules/perception/camera/lib/traffic_light/detector/detection/detection.h"
#include "modules/perception/camera/lib/traffic_light/detector/recognition/recognition.h"
#include "modules/perception/common/io/io_util.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"

namespace apollo {
namespace perception {
namespace common {
DECLARE_string(obs_sensor_meta_path);
DECLARE_string(obs_sensor_intrinsic_path);
}  // namespace common
namespace camera {

TEST(TrafficLightCameraPerceptionTest, normal) {
  unsetenv("MODULE_PATH");
  unsetenv("CYBER_PATH");
  fLS::FLAGS_obs_sensor_meta_path =
      "/apollo/modules/perception/testdata/"
      "camera/app/data/perception/camera/sensor_meta_camera.pt";
  FLAGS_obs_sensor_intrinsic_path =
      "/apollo/modules/perception/testdata/"
      "camera/app/data/perception/camera/params";
  TrafficLightCameraPerception traffic_light_perception;
  CameraPerceptionInitOptions init_options;
  CameraPerceptionOptions options;
  init_options.conf_file = "trafficlight.pt";
  init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/app/conf/perception/camera/traffic_light";
  EXPECT_TRUE(traffic_light_perception.Init(init_options));

  CameraFrame frame;
  cv::Mat origin_image = cv::imread(
      "/apollo/modules/perception/testdata/"
      "camera/app/traffic_light_images/red.jpg");
  EXPECT_TRUE(origin_image.data);
  AINFO << "Load image done";

  std::shared_ptr<base::SyncedMemory> img_gpu_data;
  int size = origin_image.cols * origin_image.rows * origin_image.channels();
  img_gpu_data.reset(new base::SyncedMemory(size, true));

  memcpy(img_gpu_data->mutable_cpu_data(), origin_image.data,
         size * sizeof(uint8_t));

  DataProvider data_provider;
  frame.data_provider = &data_provider;
  DataProvider::InitOptions dp_init_options;
  dp_init_options.image_height = origin_image.rows;
  dp_init_options.image_width = origin_image.cols;
  dp_init_options.do_undistortion = false;
  dp_init_options.sensor_name = "onsemi_narrow";
  dp_init_options.device_id = 0;
  frame.data_provider->Init(dp_init_options);
  frame.data_provider->FillImageData(
      origin_image.rows, origin_image.cols,
      (const uint8_t *)(img_gpu_data->mutable_gpu_data()), "bgr8");

  frame.traffic_lights.emplace_back(new base::TrafficLight);
  frame.traffic_lights[0]->region.projection_roi.x = 1043;
  frame.traffic_lights[0]->region.projection_roi.y = 601;
  frame.traffic_lights[0]->region.projection_roi.width = 21;
  frame.traffic_lights[0]->region.projection_roi.height = 54;

  AINFO << "lights num " << frame.traffic_lights.size();

  EXPECT_TRUE(traffic_light_perception.Perception(options, &frame));
  EXPECT_EQ(base::TLColor::TL_RED, frame.traffic_lights[0]->status.color);
}

TEST(TrafficLightCameraPerceptionTest, bad_proto) {
  unsetenv("MODULE_PATH");
  unsetenv("CYBER_PATH");
  fLS::FLAGS_obs_sensor_meta_path =
      "/apollo/modules/perception/testdata/"
      "camera/app/data/perception/camera/sensor_meta_camera.pt";
  FLAGS_obs_sensor_intrinsic_path =
      "/apollo/modules/perception/testdata/"
      "camera/app/data/perception/camera/params";
  TrafficLightCameraPerception traffic_light_perception;
  CameraPerceptionInitOptions init_options;
  init_options.conf_file = "nothing.pt";
  init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/app/conf/perception/camera/traffic_light";
  EXPECT_FALSE(traffic_light_perception.Init(init_options));
}

TEST(TrafficLightCameraPerceptionTest, bad_detector) {
  unsetenv("MODULE_PATH");
  unsetenv("CYBER_PATH");
  fLS::FLAGS_obs_sensor_meta_path =
      "/apollo/modules/perception/testdata/"
      "camera/app/data/perception/camera/sensor_meta_camera.pt";
  FLAGS_obs_sensor_intrinsic_path =
      "/apollo/modules/perception/testdata/"
      "camera/app/data/perception/camera/params";
  TrafficLightCameraPerception traffic_light_perception;
  CameraPerceptionInitOptions init_options;
  init_options.conf_file = "bad_detector_trafficlight.pt";
  init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/app/conf/perception/camera/traffic_light";
  EXPECT_FALSE(traffic_light_perception.Init(init_options));
}

TEST(TrafficLightCameraPerceptionTest, bad_recognizer) {
  unsetenv("MODULE_PATH");
  unsetenv("CYBER_PATH");
  fLS::FLAGS_obs_sensor_meta_path =
      "/apollo/modules/perception/testdata/"
      "camera/app/data/perception/camera/sensor_meta_camera.pt";
  FLAGS_obs_sensor_intrinsic_path =
      "/apollo/modules/perception/testdata/"
      "camera/app/data/perception/camera/params";
  TrafficLightCameraPerception traffic_light_perception;
  CameraPerceptionInitOptions init_options;
  init_options.conf_file = "bad_recognizer_trafficlight.pt";
  init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/app/conf/perception/camera/traffic_light";
  EXPECT_FALSE(traffic_light_perception.Init(init_options));
}

TEST(TrafficLightCameraPerceptionTest, bad_tracker) {
  unsetenv("MODULE_PATH");
  unsetenv("CYBER_PATH");
  fLS::FLAGS_obs_sensor_meta_path =
      "/apollo/modules/perception/testdata/"
      "camera/app/data/perception/camera/sensor_meta_camera.pt";
  FLAGS_obs_sensor_intrinsic_path =
      "/apollo/modules/perception/testdata/"
      "camera/app/data/perception/camera/params";
  TrafficLightCameraPerception traffic_light_perception;
  CameraPerceptionInitOptions init_options;
  init_options.conf_file = "bad_tracker_trafficlight.pt";
  init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/app/conf/perception/camera/traffic_light";
  EXPECT_FALSE(traffic_light_perception.Init(init_options));
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
