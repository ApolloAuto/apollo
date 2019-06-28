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
#include "modules/perception/camera/lib/calibration_service/online_calibration_service/online_calibration_service.h"

#include <utility>

#include "modules/perception/common/i_lib/core/i_blas.h"
#include "modules/perception/common/i_lib/core/i_constant.h"
#include "modules/perception/common/i_lib/geometry/i_util.h"

namespace apollo {
namespace perception {
namespace camera {

bool OnlineCalibrationService::Init(
    const CalibrationServiceInitOptions &options) {
  master_sensor_name_ = options.calibrator_working_sensor_name;
  sensor_name_ = options.calibrator_working_sensor_name;
  // Init k_matrix
  auto &name_intrinsic_map = options.name_intrinsic_map;
  CHECK(name_intrinsic_map.find(master_sensor_name_) !=
        name_intrinsic_map.end());
  CameraStatus camera_status;
  name_camera_status_map_.clear();
  for (auto iter = name_intrinsic_map.begin(); iter != name_intrinsic_map.end();
       ++iter) {
    camera_status.k_matrix[0] = static_cast<double>(iter->second(0, 0));
    camera_status.k_matrix[4] = static_cast<double>(iter->second(1, 1));
    camera_status.k_matrix[2] = static_cast<double>(iter->second(0, 2));
    camera_status.k_matrix[5] = static_cast<double>(iter->second(1, 2));
    camera_status.k_matrix[8] = 1.0;
    name_camera_status_map_.insert(
        std::pair<std::string, CameraStatus>(iter->first, camera_status));
  }
  // Only init calibrator on master_sensor
  CalibratorInitOptions calibrator_init_options;
  calibrator_init_options.image_width = options.image_width;
  calibrator_init_options.image_height = options.image_height;
  calibrator_init_options.focal_x = static_cast<float>(
      name_camera_status_map_[master_sensor_name_].k_matrix[0]);
  calibrator_init_options.focal_y = static_cast<float>(
      name_camera_status_map_[master_sensor_name_].k_matrix[4]);
  calibrator_init_options.cx = static_cast<float>(
      name_camera_status_map_[master_sensor_name_].k_matrix[2]);
  calibrator_init_options.cy = static_cast<float>(
      name_camera_status_map_[master_sensor_name_].k_matrix[5]);
  calibrator_.reset(
      BaseCalibratorRegisterer::GetInstanceByName(options.calibrator_method));
  CHECK(calibrator_ != nullptr);
  CHECK(calibrator_->Init(calibrator_init_options))
      << "Failed to init " << options.calibrator_method;
  return true;
}

bool OnlineCalibrationService::BuildIndex() {
  is_service_ready_ = HasSetIntrinsics() && HasSetGroundPlane();
  return is_service_ready_;
}

bool OnlineCalibrationService::QueryDepthOnGroundPlane(int x, int y,
                                                       double *depth) const {
  if (!is_service_ready_) {
    return false;
  }
  CHECK(depth != nullptr);
  double pixel[2] = {static_cast<double>(x), static_cast<double>(y)};
  double point[3] = {0};

  auto iter = name_camera_status_map_.find(sensor_name_);
  bool success = common::IBackprojectPlaneIntersectionCanonical(
      pixel, &(iter->second.k_matrix[0]), &(iter->second.ground_plane[0]),
      point);
  if (!success) {
    *depth = 0.0;
    return false;
  }

  *depth = point[2];
  return true;
}

bool OnlineCalibrationService::QueryPoint3dOnGroundPlane(
    int x, int y, Eigen::Vector3d *point3d) const {
  if (!is_service_ready_) {
    return false;
  }
  CHECK(point3d != nullptr);
  double pixel[2] = {static_cast<double>(x), static_cast<double>(y)};
  double point[3] = {0};
  auto iter = name_camera_status_map_.find(sensor_name_);
  bool success = common::IBackprojectPlaneIntersectionCanonical(
      pixel, &(iter->second.k_matrix[0]), &(iter->second.ground_plane[0]),
      point);
  if (!success) {
    (*point3d)(0) = (*point3d)(1) = (*point3d)(2) = 0.0;
    return false;
  }

  (*point3d)(0) = point[0];
  (*point3d)(1) = point[1];
  (*point3d)(2) = point[2];
  return true;
}

bool OnlineCalibrationService::QueryGroundPlaneInCameraFrame(
    Eigen::Vector4d *plane_param) const {
  CHECK(plane_param != nullptr);
  if (!is_service_ready_) {
    (*plane_param)(0) = (*plane_param)(1) = (*plane_param)(2) =
        (*plane_param)(3) = 0.0;
    return false;
  }
  auto iter = name_camera_status_map_.find(sensor_name_);
  (*plane_param)(0) = iter->second.ground_plane[0];
  (*plane_param)(1) = iter->second.ground_plane[1];
  (*plane_param)(2) = iter->second.ground_plane[2];
  (*plane_param)(3) = iter->second.ground_plane[3];
  return true;
}

bool OnlineCalibrationService::QueryCameraToGroundHeightAndPitchAngle(
    float *height, float *pitch) const {
  CHECK(height != nullptr);
  CHECK(pitch != nullptr);
  if (!is_service_ready_) {
    *height = *pitch = 0.0;
    return false;
  }
  auto iter = name_camera_status_map_.find(sensor_name_);
  *height = iter->second.camera_ground_height;
  *pitch = iter->second.pitch_angle;
  return true;
}

void OnlineCalibrationService::Update(CameraFrame *frame) {
  CHECK(frame != nullptr);
  sensor_name_ = frame->data_provider->sensor_name();
  if (sensor_name_ == master_sensor_name_) {
    CalibratorOptions calibrator_options;
    calibrator_options.lane_objects =
        std::make_shared<std::vector<base::LaneLine>>(frame->lane_objects);
    calibrator_options.camera2world_pose =
        std::make_shared<Eigen::Affine3d>(frame->camera2world_pose);
    calibrator_options.timestamp = &(frame->timestamp);
    float pitch_angle = 0.f;
    bool updated = calibrator_->Calibrate(calibrator_options, &pitch_angle);
    // rebuild the service when updated
    if (updated) {
      name_camera_status_map_[master_sensor_name_].pitch_angle = pitch_angle;
      for (auto iter = name_camera_status_map_.begin();
           iter != name_camera_status_map_.end(); iter++) {
        // update pitch angle
        iter->second.pitch_angle =
            iter->second.pitch_angle_diff + iter->second.pitch_angle;
        // update ground plane param
        iter->second.ground_plane[1] = cos(iter->second.pitch_angle);
        iter->second.ground_plane[2] = -sin(iter->second.pitch_angle);
      }
    }
  }
  auto iter = name_camera_status_map_.find(sensor_name_);
  AINFO << "camera_ground_height: " << iter->second.camera_ground_height
        << " meter.";
  AINFO << "pitch_angle: " << iter->second.pitch_angle * 180.0 / M_PI
        << " degree.";
  // CHECK(BuildIndex());
  is_service_ready_ = true;
}

void OnlineCalibrationService::SetCameraHeightAndPitch(
    const std::map<std::string, float> &name_camera_ground_height_map,
    const std::map<std::string, float> &name_camera_pitch_angle_diff_map,
    const float &pitch_angle_master_sensor) {
  name_camera_status_map_[master_sensor_name_].pitch_angle =
      pitch_angle_master_sensor;
  for (auto iter = name_camera_status_map_.begin();
       iter != name_camera_status_map_.end(); ++iter) {
    // get iters
    auto iter_ground_height = name_camera_ground_height_map.find(iter->first);
    auto iter_pitch_angle = name_camera_pitch_angle_diff_map.find(iter->first);
    auto iter_pitch_angle_diff =
        name_camera_pitch_angle_diff_map.find(iter->first);
    CHECK(iter_ground_height != name_camera_ground_height_map.end());
    CHECK(iter_pitch_angle != name_camera_pitch_angle_diff_map.end());
    CHECK(iter_pitch_angle_diff != name_camera_pitch_angle_diff_map.end());
    // set camera status
    name_camera_status_map_[iter->first].camera_ground_height =
        iter_ground_height->second;
    name_camera_status_map_[iter->first].pitch_angle_diff =
        iter_pitch_angle->second;
    name_camera_status_map_[iter->first].pitch_angle =
        pitch_angle_master_sensor + iter_pitch_angle_diff->second;
    name_camera_status_map_[iter->first].ground_plane[1] =
        cos(name_camera_status_map_[iter->first].pitch_angle);
    name_camera_status_map_[iter->first].ground_plane[2] =
        -sin(name_camera_status_map_[iter->first].pitch_angle);
    name_camera_status_map_[iter->first].ground_plane[3] =
        -name_camera_status_map_[iter->first].camera_ground_height;
  }
}

std::string OnlineCalibrationService::Name() const {
  return "OnlineCalibrationService";
}
REGISTER_CALIBRATION_SERVICE(OnlineCalibrationService);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
