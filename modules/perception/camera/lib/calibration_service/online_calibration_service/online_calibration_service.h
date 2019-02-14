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
#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "modules/perception/camera/lib/interface/base_calibration_service.h"
#include "modules/perception/camera/lib/interface/base_calibrator.h"

namespace apollo {
namespace perception {
namespace camera {

struct CameraStatus {
  float camera_ground_height = -1.f;
  float pitch_angle = 0.f;
  float pitch_angle_diff = 0.f;
  std::vector<double> k_matrix = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> ground_plane = {0.0, 0.0, 0.0, 0.0};
};

class OnlineCalibrationService : public BaseCalibrationService {
 public:
  OnlineCalibrationService() : BaseCalibrationService() {}

  virtual ~OnlineCalibrationService() {}

  bool Init(const CalibrationServiceInitOptions &options =
                CalibrationServiceInitOptions()) override;

  bool BuildIndex() override;

  // @brief query depth on ground plane given pixel coordinate
  bool QueryDepthOnGroundPlane(int x, int y, double *depth) const override;

  // @brief query 3d point on ground plane given pixel coordinate
  bool QueryPoint3dOnGroundPlane(int x, int y,
                                 Eigen::Vector3d *point3d) const override;

  // @brief query ground plane in camera frame, parameterized as
  // [n^T, d] with n^T*x+d=0
  bool QueryGroundPlaneInCameraFrame(
      Eigen::Vector4d *plane_param) const override;

  // @brief query camera to ground height and pitch angle
  bool QueryCameraToGroundHeightAndPitchAngle(float *height,
                                              float *pitch) const override;

  float QueryCameraToGroundHeight() const override {
    if (is_service_ready_) {
      auto iter = name_camera_status_map_.find(sensor_name_);
      return (iter->second).camera_ground_height;
    }
    return -1.f;
  }

  float QueryPitchAngle() const override {
    if (is_service_ready_) {
      auto iter = name_camera_status_map_.find(sensor_name_);
      return (iter->second).pitch_angle;
    }
    return -1.f;
  }

  // @brief using calibrator to update pitch angle
  void Update(CameraFrame *frame) override;

  // @brief set camera height and pitch
  void SetCameraHeightAndPitch(
      const std::map<std::string, float> &name_camera_ground_height_map,
      const std::map<std::string, float> &name_camera_pitch_angle_diff_map,
      const float &pitch_angle_master_sensor) override;

  std::string Name() const override;

 private:
  bool HasSetIntrinsics() const {
    return name_camera_status_map_.find(sensor_name_) !=
           name_camera_status_map_.end();
  }

  bool HasSetGroundPlane() {
    bool has_set_ground_plane =
        name_camera_status_map_.find(sensor_name_) !=
            name_camera_status_map_.end() &&
        name_camera_status_map_[sensor_name_].camera_ground_height > 0.0;
    return has_set_ground_plane;
  }

  bool is_service_ready_ = false;
  std::string sensor_name_ = "";
  std::string master_sensor_name_ = "";
  std::map<std::string, CameraStatus> name_camera_status_map_;
  std::shared_ptr<BaseCalibrator> calibrator_;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
