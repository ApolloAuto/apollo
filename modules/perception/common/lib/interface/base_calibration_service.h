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
#include <string>

#include "cyber/common/macros.h"
#include "modules/common/util/eigen_defs.h"
#include "modules/perception/common/camera/common/camera_frame.h"
#include "modules/perception/common/lib/interface/base_init_options.h"
#include "modules/perception/common/lib/registerer/registerer.h"
#include "modules/perception/common/onboard/inner_component_messages/camera_detection_component_messages.h"

namespace apollo {
namespace perception {
namespace camera {

struct CalibrationServiceInitOptions : public BaseInitOptions {
  int image_width = 0;
  int image_height = 0;
  double timestamp = 0;
  std::string calibrator_working_sensor_name = "";
  std::string calibrator_method = "";
  apollo::common::EigenMap<std::string, Eigen::Matrix3f> name_intrinsic_map;
};

struct CalibrationServiceOptions {};

class BaseCalibrationService {
 public:
  BaseCalibrationService() = default;

  virtual ~BaseCalibrationService() = default;

  virtual bool Init(const CalibrationServiceInitOptions &options =
                        CalibrationServiceInitOptions()) = 0;

  virtual bool BuildIndex() = 0;

  // @brief query camera to world pose with refinement if any
  virtual bool QueryCameraToWorldPose(Eigen::Matrix4d *pose) const {
    return false;
  }

  // @brief query depth on ground plane given pixel coordinate
  virtual bool QueryDepthOnGroundPlane(int x, int y, double *depth) const {
    return false;
  }

  // @brief query 3d point on ground plane given pixel coordinate
  virtual bool QueryPoint3dOnGroundPlane(int x, int y,
                                         Eigen::Vector3d *point3d) const {
    return false;
  }

  // @brief query ground plane in camera frame, parameterized as
  // [n^T, d] with n^T*x+d=0
  virtual bool QueryGroundPlaneInCameraFrame(
      Eigen::Vector4d *plane_param) const {
    return false;
  }

  // @brief query camera to ground height and pitch angle
  virtual bool QueryCameraToGroundHeightAndPitchAngle(float *height,
                                                      float *pitch) const {
    return false;
  }

  virtual float QueryCameraToGroundHeight() const { return 0.f; }

  virtual float QueryPitchAngle() const { return 0.f; }

  // @brief using calibrator to update pitch angle
  virtual void Update(onboard::CameraFrame *frame) {
    // do nothing
  }

  // @brief using calibrator to update lane detection pitch angle
  // TODO(huqilin): need to unify the camera and lane update interfaces
  virtual void Update(CameraFrame *frame) {
    // do nothing
  }

  // @brief set camera height, pitch and project matrix
  virtual void SetCameraHeightAndPitch(
      const std::map<std::string, float> &name_camera_ground_height_map,
      const std::map<std::string, float> &name_camera_pitch_angle_diff_map,
      const float &pitch_angle_master_sensor) {
    // do nothing
  }

  virtual std::string Name() const = 0;

  DISALLOW_COPY_AND_ASSIGN(BaseCalibrationService);
};  // class BaseCalibrationService

PERCEPTION_REGISTER_REGISTERER(BaseCalibrationService);
#define REGISTER_CALIBRATION_SERVICE(name) \
  PERCEPTION_REGISTER_CLASS(BaseCalibrationService, name)

}  // namespace camera
}  // namespace perception
}  // namespace apollo
