/******************************************************************************
 * Copyright 2026 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/camera_location_refinement/camera_location_refinement_component.h"

#include "gtest/gtest.h"

namespace apollo {
namespace perception {
namespace camera {

class RecordingCalibrationService : public BaseCalibrationService {
 public:
  bool Init(const CalibrationServiceInitOptions &options =
                CalibrationServiceInitOptions()) override {
    return true;
  }

  bool BuildIndex() override {
    build_index_called = true;
    return build_index_result;
  }

  void SetCameraHeightAndPitch(
      const std::map<std::string, float> &name_camera_ground_height_map,
      const std::map<std::string, float> &name_camera_pitch_angle_diff_map,
      const float &pitch_angle_master_sensor) override {
    camera_ground_height_map = name_camera_ground_height_map;
    camera_pitch_angle_diff_map = name_camera_pitch_angle_diff_map;
    master_sensor_pitch = pitch_angle_master_sensor;
  }

  std::string Name() const override { return "RecordingCalibrationService"; }

  bool build_index_called = false;
  bool build_index_result = true;
  float master_sensor_pitch = 0.0f;
  std::map<std::string, float> camera_ground_height_map;
  std::map<std::string, float> camera_pitch_angle_diff_map;
};

class CameraLocationRefinementComponentTest : public ::testing::Test {};

TEST_F(CameraLocationRefinementComponentTest,
       init_static_calibration_builds_ground_plane_test) {
  CameraLocationRefinementComponent component;
  auto calibration_service = std::make_shared<RecordingCalibrationService>();
  component.calibration_service_ = calibration_service;

  CameraLocationRefinement config;
  config.set_camera_name("front_6mm");
  config.set_default_camera_pitch(0.12f);
  config.set_default_camera_height(1.8f);

  EXPECT_TRUE(component.InitStaticCalibration(config));
  EXPECT_TRUE(calibration_service->build_index_called);
  ASSERT_EQ(calibration_service->camera_ground_height_map.size(), 1u);
  ASSERT_EQ(calibration_service->camera_pitch_angle_diff_map.size(), 1u);
  EXPECT_FLOAT_EQ(
      calibration_service->camera_ground_height_map.at("front_6mm"), 1.8f);
  EXPECT_FLOAT_EQ(
      calibration_service->camera_pitch_angle_diff_map.at("front_6mm"), 0.0f);
  EXPECT_FLOAT_EQ(calibration_service->master_sensor_pitch, 0.12f);
}

TEST_F(CameraLocationRefinementComponentTest,
       init_static_calibration_rejects_non_positive_height_test) {
  CameraLocationRefinementComponent component;
  auto calibration_service = std::make_shared<RecordingCalibrationService>();
  component.calibration_service_ = calibration_service;

  CameraLocationRefinement config;
  config.set_camera_name("front_6mm");
  config.set_default_camera_height(0.0f);

  EXPECT_FALSE(component.InitStaticCalibration(config));
  EXPECT_FALSE(calibration_service->build_index_called);
  EXPECT_TRUE(calibration_service->camera_ground_height_map.empty());
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
