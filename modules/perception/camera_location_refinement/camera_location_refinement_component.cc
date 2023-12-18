/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include <utility>

#include "cyber/common/file.h"
#include "cyber/profiler/profiler.h"
#include "modules/perception/common/algorithm/sensor_manager/sensor_manager.h"
#include "modules/perception/common/base/camera.h"

namespace apollo {
namespace perception {
namespace camera {

void CameraLocationRefinementComponent::InitCalibrationService(
    const CameraLocationRefinement& location_refinement_param) {
  base::BaseCameraModelPtr model =
      algorithm::SensorManager::Instance()->GetUndistortCameraModel(
          location_refinement_param.camera_name());
  ACHECK(model) << "Can't find " << location_refinement_param.camera_name()
                << " in data/conf/sensor_meta.pb.txt";
  auto pinhole = static_cast<base::PinholeCameraModel*>(model.get());
  name_intrinsic_map_.insert(std::pair<std::string, Eigen::Matrix3f>(
      location_refinement_param.camera_name(),
      pinhole->get_intrinsic_params()));

  auto calibration_service_param =
      location_refinement_param.calibration_service_param();
  CalibrationServiceInitOptions calibration_service_init_options;
  calibration_service_init_options.calibrator_working_sensor_name =
      location_refinement_param.camera_name();
  calibration_service_init_options.name_intrinsic_map = name_intrinsic_map_;
  calibration_service_init_options.calibrator_method =
      calibration_service_param.calibrator_method();
  calibration_service_init_options.image_height =
      static_cast<int>(model->get_height());
  calibration_service_init_options.image_width =
      static_cast<int>(model->get_width());
  calibration_service_.reset(
      BaseCalibrationServiceRegisterer::GetInstanceByName(
          calibration_service_param.plugin_param().name()));
  ACHECK(calibration_service_ != nullptr);
  ACHECK(calibration_service_->Init(calibration_service_init_options))
      << "Failed to init: " << calibration_service_param.plugin_param().name();
  AINFO << "calibration_service: " << calibration_service_->Name();
}

void CameraLocationRefinementComponent::InitPostprocessor(
    const CameraLocationRefinement& location_refinement_param) {
  InitCalibrationService(location_refinement_param);

  PostprocessorInitOptions postprocessor_init_options;
  auto plugin_param =
      location_refinement_param.postprocessor_param().plugin_param();
  postprocessor_init_options.config_path = plugin_param.config_path();
  postprocessor_init_options.config_file = plugin_param.config_file();
  postprocessor_init_options.calibration_service = calibration_service_;
  postprocessor_.reset(
      BasePostprocessorRegisterer::GetInstanceByName(plugin_param.name()));
  ACHECK(postprocessor_ != nullptr);
  ACHECK(postprocessor_->Init(postprocessor_init_options))
      << "Failed to init: " << plugin_param.name();
}

void CameraLocationRefinementComponent::SetCameraHeightAndPitch(
    const std::map<std::string, float>& name_camera_ground_height_map,
    const std::map<std::string, float>& name_camera_pitch_angle_diff_map,
    const float& pitch_angle_calibrator_working_sensor) {
  if (calibration_service_ == nullptr) {
    AERROR << "Calibraion service is not available";
    return;
  }
  calibration_service_->SetCameraHeightAndPitch(
      name_camera_ground_height_map, name_camera_pitch_angle_diff_map,
      pitch_angle_calibrator_working_sensor);
}

bool CameraLocationRefinementComponent::Init() {
  CameraLocationRefinement location_refinement_param;
  if (!GetProtoConfig(&location_refinement_param)) {
    AERROR << "Load camera detection 3d component config failed!";
    return false;
  }

  InitPostprocessor(location_refinement_param);

  // todo(daohu527): need complete
  // SetCameraHeightAndPitch();

  writer_ = node_->CreateWriter<onboard::CameraFrame>(
      location_refinement_param.channel().output_obstacles_channel_name());
  return true;
}

bool CameraLocationRefinementComponent::Proc(
    const std::shared_ptr<onboard::CameraFrame>& msg) {
  PERF_FUNCTION()
  PostprocessorOptions obstacle_postprocessor_options;
  obstacle_postprocessor_options.do_refinement_with_calibration_service =
      calibration_service_ != nullptr;

  std::shared_ptr<onboard::CameraFrame> out_message(new (std::nothrow)
                                                        onboard::CameraFrame);
  out_message->frame_id = msg->frame_id;
  out_message->timestamp = msg->timestamp;
  out_message->data_provider = msg->data_provider;
  out_message->detected_objects = msg->detected_objects;
  out_message->feature_blob = msg->feature_blob;
  out_message->camera_k_matrix = msg->camera_k_matrix;
  out_message->camera2world_pose = msg->camera2world_pose;

  // calibration_service_->Update(&frame);

  PERF_BLOCK("location_refinement_postprocessor")
  if (!postprocessor_->Process(obstacle_postprocessor_options,
                               out_message.get())) {
    AERROR << "Failed to post process obstacles.";
    return false;
  }
  PERF_BLOCK_END

  writer_->Write(out_message);
  return true;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
