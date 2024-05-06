/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/lane_detection/app/lane_camera_perception.h"

#include <algorithm>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "gflags/gflags.h"
#include "yaml-cpp/yaml.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/util/perf_util.h"
#include "modules/perception/common/algorithm/io/io_util.h"
#include "modules/perception/common/algorithm/sensor_manager/sensor_manager.h"
#include "modules/perception/common/base/object.h"
#include "modules/perception/common/camera/common/global_config.h"
#include "modules/perception/common/inference/utils/cuda_util.h"
#include "modules/perception/common/util.h"
#include "modules/perception/lane_detection/app/debug_info.h"

namespace apollo {
namespace perception {
namespace camera {

using cyber::common::EnsureDirectory;
using cyber::common::GetAbsolutePath;

bool LaneCameraPerception::Init(const CameraPerceptionInitOptions &options) {
  std::string config_file =
      GetConfigFile(options.config_path, options.config_file);
  ACHECK(cyber::common::GetProtoFromFile(config_file, &perception_param_))
      << "Read config failed: " << config_file;
  ACHECK(inference::CudaUtil::set_device_id(perception_param_.gpu_id()));

  lane_calibration_working_sensor_name_ =
      options.lane_calibration_working_sensor_name;

  // Init lane
  base::BaseCameraModelPtr model;
  InitLane(model, perception_param_);

  // Init calibration service
  InitCalibrationService(model, perception_param_);

  return true;
}

void LaneCameraPerception::InitLane(
    base::BaseCameraModelPtr &model,
    const app::PerceptionParam &perception_param) {
  // Init lane
  CHECK_GT(perception_param.lane_param_size(), 0)
      << "Failed to include lane_param";
  for (int i = 0; i < perception_param.lane_param_size(); ++i) {
    // Initialize lane detector
    const auto &lane_param = perception_param.lane_param(i);
    ACHECK(lane_param.has_lane_detector_param())
        << "Failed to include lane_detector_param.";
    LaneDetectorInitOptions lane_detector_init_options;
    const auto &lane_detector_param = lane_param.lane_detector_param();
    const auto &lane_detector_plugin_param = lane_detector_param.plugin_param();
    lane_detector_init_options.config_file =
        lane_detector_plugin_param.config_file();
    lane_detector_init_options.config_path =
        lane_detector_plugin_param.config_path();
    model = algorithm::SensorManager::Instance()->GetUndistortCameraModel(
        lane_detector_param.camera_name());
    ACHECK(model) << "Can't find " << lane_detector_param.camera_name()
                  << " in data/conf/sensor_meta.pb.txt";
    auto pinhole = static_cast<base::PinholeCameraModel *>(model.get());
    name_intrinsic_map_.insert(std::pair<std::string, Eigen::Matrix3f>(
        lane_detector_param.camera_name(), pinhole->get_intrinsic_params()));
    lane_detector_init_options.gpu_id = perception_param.gpu_id();
    lane_detector_init_options.base_camera_model = model;
    AINFO << "lane_detector_name: " << lane_detector_plugin_param.name();

    // Initialize lane detector
    lane_detector_.reset(BaseLaneDetectorRegisterer::GetInstanceByName(
        lane_detector_plugin_param.name()));
    CHECK_NOTNULL(lane_detector_);
    ACHECK(lane_detector_->Init(lane_detector_init_options))
        << "Failed to init: " << lane_detector_plugin_param.name();
    AINFO << "Detector: " << lane_detector_->Name();

    // Initialize lane postprocessor
    const auto &lane_postprocessor_param =
        lane_param.lane_postprocessor_param();
    LanePostprocessorInitOptions postprocessor_init_options;
    postprocessor_init_options.config_path =
        lane_postprocessor_param.config_path();
    postprocessor_init_options.config_file =
        lane_postprocessor_param.config_file();

    lane_postprocessor_.reset(
        BaseLanePostprocessorRegisterer::GetInstanceByName(
            lane_postprocessor_param.name()));
    CHECK_NOTNULL(lane_postprocessor_);
    ACHECK(lane_postprocessor_->Init(postprocessor_init_options))
        << "Failed to init: " << lane_postprocessor_param.name();
    AINFO << "lane_postprocessor: " << lane_postprocessor_->Name();

    // Init output file folder
    if (perception_param.has_debug_param() &&
        perception_param.debug_param().has_lane_out_dir()) {
      write_out_lane_file_ = true;
      out_lane_dir_ = perception_param.debug_param().lane_out_dir();
      EnsureDirectory(out_lane_dir_);
    }

    if (perception_param.has_debug_param() &&
        perception_param.debug_param().has_calibration_out_dir()) {
      write_out_calib_file_ = true;
      out_calib_dir_ = perception_param.debug_param().calibration_out_dir();
      EnsureDirectory(out_calib_dir_);
    }
  }
}

void LaneCameraPerception::InitCalibrationService(
    const base::BaseCameraModelPtr model,
    const app::PerceptionParam &perception_param) {
  // Init calibration service
  ACHECK(perception_param.has_calibration_service_param())
      << "Failed to include calibration_service_param";

  auto calibration_service_param = perception_param.calibration_service_param();
  CalibrationServiceInitOptions calibration_service_init_options;
  calibration_service_init_options.calibrator_working_sensor_name =
      lane_calibration_working_sensor_name_;
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
  CHECK_NOTNULL(calibration_service_);
  ACHECK(calibration_service_->Init(calibration_service_init_options))
      << "Failed to init " << calibration_service_param.plugin_param().name();
  AINFO << "Calibration service: " << calibration_service_->Name();
}

void LaneCameraPerception::SetCameraHeightAndPitch(
    const std::map<std::string, float> name_camera_ground_height_map,
    const std::map<std::string, float> name_camera_pitch_angle_diff_map,
    const float &pitch_angle_calibrator_working_sensor) {
  if (calibration_service_ == nullptr) {
    AERROR << "Calibraion service is not available";
    return;
  }
  calibration_service_->SetCameraHeightAndPitch(
      name_camera_ground_height_map, name_camera_pitch_angle_diff_map,
      pitch_angle_calibrator_working_sensor);
}

void LaneCameraPerception::SetIm2CarHomography(
    const Eigen::Matrix3d &homography_im2car) {
  if (calibration_service_ == nullptr) {
    AERROR << "Calibraion service is not available";
    return;
  }
  lane_postprocessor_->SetIm2CarHomography(homography_im2car);
}

bool LaneCameraPerception::GetCalibrationService(
    BaseCalibrationService **calibration_service) {
  *calibration_service = calibration_service_.get();
  return true;
}

bool LaneCameraPerception::Perception(const CameraPerceptionOptions &options,
                                      CameraFrame *frame) {
  PERF_FUNCTION();
  inference::CudaUtil::set_device_id(perception_param_.gpu_id());
  PERF_BLOCK_START();

  if (frame->calibration_service == nullptr) {
    AERROR << "Calibraion service is not available";
    return false;
  }

  // Lane detector and postprocessor: work on front_6mm only
  if (lane_calibration_working_sensor_name_ ==
      frame->data_provider->sensor_name()) {
    frame->camera_k_matrix =
        name_intrinsic_map_.at(frame->data_provider->sensor_name());
    LaneDetectorOptions lane_detetor_options;
    LanePostprocessorOptions lane_postprocessor_options;
    if (!lane_detector_->Detect(lane_detetor_options, frame)) {
      AERROR << "Failed to detect lane.";
      return false;
    }
    PERF_BLOCK_END_WITH_INDICATOR(frame->data_provider->sensor_name(),
                                  "LaneDetector");

    if (!lane_postprocessor_->Process2D(lane_postprocessor_options, frame)) {
      AERROR << "Failed to postprocess lane 2D.";
      return false;
    }
    PERF_BLOCK_END_WITH_INDICATOR(frame->data_provider->sensor_name(),
                                  "LanePostprocessor2D");

    // Calibration service
    frame->calibration_service->Update(frame);
    PERF_BLOCK_END_WITH_INDICATOR(frame->data_provider->sensor_name(),
                                  "CalibrationService");

    if (!lane_postprocessor_->Process3D(lane_postprocessor_options, frame)) {
      AERROR << "Failed to postprocess lane 3D.";
      return false;
    }
    PERF_BLOCK_END_WITH_INDICATOR(frame->data_provider->sensor_name(),
                                  "LanePostprocessor3D");

    if (write_out_lane_file_) {
      std::string lane_file_path =
          absl::StrCat(out_lane_dir_, "/", frame->frame_id, ".txt");
      WriteLanelines(write_out_lane_file_, lane_file_path, frame->lane_objects);
    }
  } else {
    AINFO << "Skip lane detection & calibration due to sensor mismatch.";
    AINFO << "Will use service sync from obstacle camera instead.";
    // fill the frame using previous estimates
    frame->calibration_service->Update(frame);
    PERF_BLOCK_END_WITH_INDICATOR(frame->data_provider->sensor_name(),
                                  "CalibrationService");
  }

  if (write_out_calib_file_) {
    std::string calib_file_path =
        absl::StrCat(out_calib_dir_, "/", frame->frame_id, ".txt");
    WriteCalibrationOutput(write_out_calib_file_, calib_file_path, frame);
  }
  return true;
}

PERCEPTION_REGISTER_CAMERA_PERCEPTION(LaneCameraPerception);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
