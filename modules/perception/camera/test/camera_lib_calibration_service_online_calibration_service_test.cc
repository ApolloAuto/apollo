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
#include "gtest/gtest.h"

#include "cyber/common/log.h"
#include "modules/perception/base/distortion_model.h"
#include "modules/perception/camera/lib/calibration_service/online_calibration_service/online_calibration_service.h"  // NOLINT
#include "modules/perception/common/io/io_util.h"

namespace apollo {
namespace perception {
namespace camera {

TEST(OnlineCalibrationServiceTest, online_calibration_service_test) {
  base::BrownCameraDistortionModel model;
  common::LoadBrownCameraIntrinsic(
      "/apollo/modules/perception/testdata/"
      "camera/lib/calibration_service/online_calibration_service/"
      "params/onsemi_obstacle_intrinsics.yaml",
      &model);
  auto pinhole =
      static_cast<base::PinholeCameraModel *>(model.get_camera_model().get());
  Eigen::Matrix3f intrinsic = pinhole->get_intrinsic_params();

  // service
  CalibrationServiceInitOptions options;
  std::map<std::string, Eigen::Matrix3f> name_intrinsic_map;
  name_intrinsic_map.insert(
      std::pair<std::string, Eigen::Matrix3f>("onsemi_obstacle", intrinsic));
  options.name_intrinsic_map = name_intrinsic_map;
  options.calibrator_working_sensor_name = "onsemi_obstacle";
  options.calibrator_method = "LaneLineCalibrator";
  options.image_height = 1080;
  options.image_width = 1920;
  BaseCalibrationService *calibration_service =
      BaseCalibrationServiceRegisterer::GetInstanceByName(
          "OnlineCalibrationService");

  EXPECT_EQ(calibration_service->Name(), "OnlineCalibrationService");
  EXPECT_TRUE(calibration_service->Init(options));

  const double principal_x = intrinsic(0, 2);
  const double principal_y = intrinsic(1, 2);
  const double camera_ground_height = 1.5;
  std::map<std::string, float> name_camera_ground_height_map;
  std::map<std::string, float> name_camera_pitch_angle_diff_map;
  name_camera_ground_height_map["onsemi_obstacle"] = camera_ground_height;
  name_camera_pitch_angle_diff_map["onsemi_obstacle"] = 0.f;

  Eigen::Vector4d plane;
  OnlineCalibrationService *online_calib_service =
      dynamic_cast<OnlineCalibrationService *>(calibration_service);
  if (online_calib_service != nullptr) {
    float camera_ground_height_query = 0.0f;
    float camera_pitch_angle_query = 0.0f;
    EXPECT_FALSE(online_calib_service->BuildIndex());
    EXPECT_FALSE(online_calib_service->QueryCameraToGroundHeightAndPitchAngle(
        &camera_ground_height_query, &camera_pitch_angle_query));
    EXPECT_FALSE(online_calib_service->QueryGroundPlaneInCameraFrame(&plane));

    EXPECT_FALSE(online_calib_service->BuildIndex());

    online_calib_service->SetCameraHeightAndPitch(
        name_camera_ground_height_map, name_camera_pitch_angle_diff_map, 0.f);
    EXPECT_TRUE(online_calib_service->BuildIndex());
    EXPECT_TRUE(online_calib_service->QueryGroundPlaneInCameraFrame(&plane));

    int x = static_cast<int>(principal_x);
    int y = static_cast<int>(principal_y) + 200;

    Eigen::Vector3d point3d;
    EXPECT_TRUE(
        online_calib_service->QueryPoint3dOnGroundPlane(x, y, &point3d));

    EXPECT_TRUE(online_calib_service->QueryCameraToGroundHeightAndPitchAngle(
        &camera_ground_height_query, &camera_pitch_angle_query));

    double depth = 0.0;
    EXPECT_TRUE(online_calib_service->QueryDepthOnGroundPlane(x, y, &depth));
    EXPECT_GT(depth, 0.0);
    AINFO << "Query depth: " << depth;

    name_camera_ground_height_map["onsemi_obstacle"] = 100;
    name_camera_pitch_angle_diff_map["onsemi_obstacle"] = 0;

    online_calib_service->SetCameraHeightAndPitch(
        name_camera_ground_height_map, name_camera_pitch_angle_diff_map, 0.f);
    EXPECT_TRUE(
        online_calib_service->QueryPoint3dOnGroundPlane(x, y, &point3d));
  }

  delete calibration_service;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
