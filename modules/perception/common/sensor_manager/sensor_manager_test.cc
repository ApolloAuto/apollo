/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#define private public
#define protected public
#include "modules/perception/common/sensor_manager/sensor_manager.h"

namespace apollo {
namespace perception {
namespace common {

/* TODO(all): to be added back
TEST(SensorManagerTest, test) {
  FLAGS_work_root = "/apollo/modules/perception/testdata/"
      "common/sensor_manager/";
  FLAGS_obs_sensor_meta_path = "data/sensor_meta.pt";
  FLAGS_obs_sensor_intrinsic_path =
      "modules/perception/testdata/common/sensor_manager/params";
  SensorManager* sensor_manager = SensorManager::Instance();

  EXPECT_TRUE(sensor_manager->Init());
  EXPECT_EQ(sensor_manager->sensor_info_map_.size(), 5);
  EXPECT_EQ(sensor_manager->distort_model_map_.size(), 1);
  EXPECT_EQ(sensor_manager->undistort_model_map_.size(), 1);

  std::string correct_name = "camera_smartereye";
  std::string wrong_name = "camera_dummy";
  base::SensorInfo sensor_info;

  EXPECT_TRUE(sensor_manager->IsSensorExist(correct_name));
  EXPECT_FALSE(sensor_manager->IsSensorExist(wrong_name));

  EXPECT_FALSE(sensor_manager->GetSensorInfo(correct_name, nullptr));
  EXPECT_FALSE(sensor_manager->GetSensorInfo(wrong_name, &sensor_info));
  EXPECT_TRUE(sensor_manager->GetSensorInfo(correct_name, &sensor_info));

  std::shared_ptr<BaseCameraDistortionModel> distort_model =
      sensor_manager->GetDistortCameraModel(correct_name);
  EXPECT_NE(distort_model, nullptr);
  EXPECT_EQ(distort_model->name(), "BrownCameraDistortionModel");
  EXPECT_EQ(sensor_manager->GetDistortCameraModel(wrong_name), nullptr);

  base::BaseCameraModelPtr undistort_model =
      sensor_manager->GetUndistortCameraModel(correct_name);
  EXPECT_NE(undistort_model, nullptr);
  EXPECT_EQ(undistort_model->name(), "PinholeCameraModel");
  EXPECT_EQ(sensor_manager->GetUndistortCameraModel(wrong_name), nullptr);

  std::string hdlidar_name = "velodyne64";
  std::string ldlidar_name = "ldlidar4";
  EXPECT_TRUE(sensor_manager->IsHdLidar(hdlidar_name));
  EXPECT_FALSE(sensor_manager->IsHdLidar(ldlidar_name));
  EXPECT_FALSE(sensor_manager->IsHdLidar(wrong_name));
  EXPECT_TRUE(sensor_manager->IsLdLidar(ldlidar_name));
  EXPECT_FALSE(sensor_manager->IsLdLidar(hdlidar_name));
  EXPECT_FALSE(sensor_manager->IsLdLidar(wrong_name));

  EXPECT_TRUE(sensor_manager->IsHdLidar(base::SensorType::VELODYNE_64));
  EXPECT_FALSE(sensor_manager->IsHdLidar(base::SensorType::LDLIDAR_4));
  EXPECT_FALSE(sensor_manager->IsLdLidar(base::SensorType::VELODYNE_64));
  EXPECT_TRUE(sensor_manager->IsLdLidar(base::SensorType::LDLIDAR_4));

  std::string lidar_name = "velodyne64";
  std::string radar_name = "radar";
  std::string camera_name = "camera_smartereye";
  std::string ultrasonic_name = "ultrasonic";
  EXPECT_TRUE(sensor_manager->IsLidar(lidar_name));
  EXPECT_FALSE(sensor_manager->IsLidar(radar_name));
  EXPECT_FALSE(sensor_manager->IsLidar(wrong_name));
  EXPECT_TRUE(sensor_manager->IsRadar(radar_name));
  EXPECT_FALSE(sensor_manager->IsRadar(lidar_name));
  EXPECT_FALSE(sensor_manager->IsRadar(wrong_name));
  EXPECT_TRUE(sensor_manager->IsCamera(camera_name));
  EXPECT_FALSE(sensor_manager->IsCamera(lidar_name));
  EXPECT_FALSE(sensor_manager->IsCamera(wrong_name));
  EXPECT_TRUE(sensor_manager->IsUltrasonic(ultrasonic_name));
  EXPECT_FALSE(sensor_manager->IsUltrasonic(lidar_name));
  EXPECT_FALSE(sensor_manager->IsUltrasonic(wrong_name));

  EXPECT_TRUE(sensor_manager->IsLidar(base::SensorType::VELODYNE_64));
  EXPECT_FALSE(sensor_manager->IsLidar(base::SensorType::LONG_RANGE_RADAR));
  EXPECT_TRUE(sensor_manager->IsRadar(base::SensorType::LONG_RANGE_RADAR));
  EXPECT_FALSE(sensor_manager->IsRadar(base::SensorType::VELODYNE_64));
  EXPECT_TRUE(sensor_manager->IsCamera(base::SensorType::STEREO_CAMERA));
  EXPECT_FALSE(sensor_manager->IsCamera(base::SensorType::LDLIDAR_4));
  EXPECT_TRUE(sensor_manager->IsUltrasonic(base::SensorType::ULTRASONIC));
  EXPECT_FALSE(sensor_manager->IsUltrasonic(base::SensorType::VELODYNE_64));

  EXPECT_EQ(sensor_manager->GetFrameId(lidar_name), "velodyne64");
  EXPECT_EQ(sensor_manager->GetFrameId(wrong_name), "");
}
*/

/* TODO(all): test not working. to be added back
TEST(SensorManagerTest, test_init_error) {
  SensorManager* sensor_manager = SensorManager::Instance();
  sensor_manager->inited_ = false;
  FLAGS_work_root = "/apollo/modules/perception/testdata/"
      "common/sensor_manager/";

  FLAGS_obs_sensor_meta_path = "./data/sensor_meta_error.pt";
  EXPECT_FALSE(sensor_manager->Init());

  FLAGS_obs_sensor_meta_path = "./data/sensor_meta_error_1.pt";
  EXPECT_FALSE(sensor_manager->Init());

  FLAGS_obs_sensor_meta_path = "./data/sensor_meta_error_2.pt";
  EXPECT_FALSE(sensor_manager->Init());
}
*/

}  // namespace common
}  // namespace perception
}  // namespace apollo
