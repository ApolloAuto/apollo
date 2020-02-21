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
#include "modules/perception/camera/app/obstacle_camera_perception.h"

#include <utility>

#include "absl/strings/str_cat.h"
#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/base/object.h"
#include "modules/perception/camera/app/debug_info.h"
#include "modules/perception/camera/common/global_config.h"
#include "modules/perception/camera/common/util.h"
#include "modules/perception/common/io/io_util.h"
#include "modules/perception/inference/utils/cuda_util.h"
#include "modules/perception/lib/utils/perf.h"

namespace apollo {
namespace perception {
namespace camera {

using cyber::common::EnsureDirectory;
using cyber::common::GetAbsolutePath;

bool ObstacleCameraPerception::Init(
    const CameraPerceptionInitOptions &options) {
  std::string work_root = "";
  if (options.use_cyber_work_root) {
    work_root = GetCyberWorkRoot();
  }
  std::string config_file =
      GetAbsolutePath(options.root_dir, options.conf_file);
  config_file = GetAbsolutePath(work_root, config_file);
  ACHECK(cyber::common::GetProtoFromFile(config_file, &perception_param_))
      << "Read config failed: " << config_file;
  ACHECK(inference::CudaUtil::set_device_id(perception_param_.gpu_id()));

  // Init detector
  ACHECK(perception_param_.detector_param_size() > 0)
      << "Failed to init detector.";
  // Init detector
  base::BaseCameraModelPtr model;
  for (int i = 0; i < perception_param_.detector_param_size(); ++i) {
    ObstacleDetectorInitOptions detector_init_options;
    app::DetectorParam detector_param = perception_param_.detector_param(i);
    auto plugin_param = detector_param.plugin_param();
    detector_init_options.root_dir =
        GetAbsolutePath(work_root, plugin_param.root_dir());
    detector_init_options.conf_file = plugin_param.config_file();
    detector_init_options.gpu_id = perception_param_.gpu_id();

    model = common::SensorManager::Instance()->GetUndistortCameraModel(
        detector_param.camera_name());
    auto pinhole = static_cast<base::PinholeCameraModel *>(model.get());
    name_intrinsic_map_.insert(std::pair<std::string, Eigen::Matrix3f>(
        detector_param.camera_name(), pinhole->get_intrinsic_params()));
    detector_init_options.base_camera_model = model;
    std::shared_ptr<BaseObstacleDetector> detector_ptr(
        BaseObstacleDetectorRegisterer::GetInstanceByName(plugin_param.name()));
    name_detector_map_.insert(
        std::pair<std::string, std::shared_ptr<BaseObstacleDetector>>(
            detector_param.camera_name(), detector_ptr));
    ACHECK(name_detector_map_.at(detector_param.camera_name()) != nullptr);
    ACHECK(name_detector_map_.at(detector_param.camera_name())
               ->Init(detector_init_options))
        << "Failed to init: " << plugin_param.name();
  }

  // Init tracker
  ACHECK(perception_param_.has_tracker_param()) << "Failed to init tracker.";
  {
    ObstacleTrackerInitOptions tracker_init_options;
    tracker_init_options.image_width = static_cast<float>(model->get_width());
    tracker_init_options.image_height = static_cast<float>(model->get_height());
    tracker_init_options.gpu_id = perception_param_.gpu_id();
    auto plugin_param = perception_param_.tracker_param().plugin_param();
    tracker_init_options.root_dir =
        GetAbsolutePath(work_root, plugin_param.root_dir());
    tracker_init_options.conf_file = plugin_param.config_file();
    tracker_.reset(
        BaseObstacleTrackerRegisterer::GetInstanceByName(plugin_param.name()));
    ACHECK(tracker_ != nullptr);
    ACHECK(tracker_->Init(tracker_init_options))
        << "Failed to init: " << plugin_param.name();
  }

  // Init transformer
  ACHECK(perception_param_.has_transformer_param())
      << "Failed to init transformer.";
  {
    ObstacleTransformerInitOptions transformer_init_options;
    auto plugin_param = perception_param_.transformer_param().plugin_param();
    transformer_init_options.root_dir =
        GetAbsolutePath(work_root, plugin_param.root_dir());
    transformer_init_options.conf_file = plugin_param.config_file();
    transformer_.reset(BaseObstacleTransformerRegisterer::GetInstanceByName(
        plugin_param.name()));
    ACHECK(transformer_ != nullptr);
    ACHECK(transformer_->Init(transformer_init_options))
        << "Failed to init: " << plugin_param.name();
  }

  // Init obstacle postprocessor
  ACHECK(perception_param_.has_postprocessor_param())
      << "Failed to init obstacle postprocessor.";
  {
    ObstaclePostprocessorInitOptions obstacle_postprocessor_init_options;
    auto plugin_param = perception_param_.postprocessor_param().plugin_param();
    obstacle_postprocessor_init_options.root_dir =
        GetAbsolutePath(work_root, plugin_param.root_dir());
    obstacle_postprocessor_init_options.conf_file = plugin_param.config_file();
    obstacle_postprocessor_.reset(
        BaseObstaclePostprocessorRegisterer::GetInstanceByName(
            plugin_param.name()));
    ACHECK(obstacle_postprocessor_ != nullptr);
    ACHECK(obstacle_postprocessor_->Init(obstacle_postprocessor_init_options))
        << "Failed to init: " << plugin_param.name();
  }

  // Init feature_extractor
  if (!perception_param_.has_feature_param()) {
    AINFO << "No feature config found.";
    extractor_ = nullptr;
  } else {
    FeatureExtractorInitOptions init_options;
    auto plugin_param = perception_param_.feature_param().plugin_param();
    init_options.root_dir = GetAbsolutePath(work_root, plugin_param.root_dir());
    init_options.conf_file = plugin_param.config_file();
    extractor_.reset(
        BaseFeatureExtractorRegisterer::GetInstanceByName(plugin_param.name()));
    ACHECK(extractor_ != nullptr);
    ACHECK(extractor_->Init(init_options))
        << "Failed to init: " << plugin_param.name();
  }

  lane_calibration_working_sensor_name_ =
      options.lane_calibration_working_sensor_name;

  // Init lane
  InitLane(work_root, perception_param_);

  // Init calibration service
  InitCalibrationService(work_root, model, perception_param_);

  // Init debug_param
  if (perception_param_.has_debug_param()) {
    // Init debug info
    if (perception_param_.debug_param().has_track_out_file()) {
      out_track_.open(perception_param_.debug_param().track_out_file(),
                      std::ofstream::out);
    }
    if (perception_param_.debug_param().has_camera2world_out_file()) {
      out_pose_.open(perception_param_.debug_param().camera2world_out_file(),
                     std::ofstream::out);
    }
  }

  // Init object template
  if (perception_param_.has_object_template_param()) {
    ObjectTemplateManagerInitOptions init_options;
    auto plugin_param =
        perception_param_.object_template_param().plugin_param();
    init_options.root_dir = GetAbsolutePath(work_root, plugin_param.root_dir());
    init_options.conf_file = plugin_param.config_file();
    ACHECK(ObjectTemplateManager::Instance()->Init(init_options));
  }
  return true;
}

void ObstacleCameraPerception::InitLane(
    const std::string &work_root,
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
    lane_detector_init_options.conf_file =
        lane_detector_plugin_param.config_file();
    lane_detector_init_options.root_dir =
        GetAbsolutePath(work_root, lane_detector_plugin_param.root_dir());
    lane_detector_init_options.gpu_id = perception_param.gpu_id();
    base::BaseCameraModelPtr model =
        common::SensorManager::Instance()->GetUndistortCameraModel(
            lane_detector_param.camera_name());
    lane_detector_init_options.base_camera_model = model;
    AINFO << "lane_detector_name: " << lane_detector_plugin_param.name();
    lane_detector_.reset(BaseLaneDetectorRegisterer::GetInstanceByName(
        lane_detector_plugin_param.name()));
    ACHECK(lane_detector_ != nullptr);
    ACHECK(lane_detector_->Init(lane_detector_init_options))
        << "Failed to init: " << lane_detector_plugin_param.name();
    AINFO << "Detector: " << lane_detector_->Name();

    // Initialize lane postprocessor
    const auto &lane_postprocessor_param =
        lane_param.lane_postprocessor_param();
    LanePostprocessorInitOptions postprocessor_init_options;
    postprocessor_init_options.detect_config_root =
        GetAbsolutePath(work_root, lane_detector_plugin_param.root_dir());
    postprocessor_init_options.detect_config_name =
        lane_detector_plugin_param.config_file();
    postprocessor_init_options.root_dir =
        GetAbsolutePath(work_root, lane_postprocessor_param.root_dir());
    postprocessor_init_options.conf_file =
        lane_postprocessor_param.config_file();
    lane_postprocessor_.reset(
        BaseLanePostprocessorRegisterer::GetInstanceByName(
            lane_postprocessor_param.name()));
    ACHECK(lane_postprocessor_ != nullptr);
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

void ObstacleCameraPerception::InitCalibrationService(
    const std::string &work_root, const base::BaseCameraModelPtr model,
    const app::PerceptionParam &perception_param) {
  // Init calibration service
  ACHECK(perception_param.has_calibration_service_param())
      << "Failed to include calibration_service_param.";
  {
    auto calibration_service_param =
        perception_param.calibration_service_param();
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
    ACHECK(calibration_service_ != nullptr);
    ACHECK(calibration_service_->Init(calibration_service_init_options))
        << "Failed to init: "
        << calibration_service_param.plugin_param().name();
    AINFO << "calibration_service: " << calibration_service_->Name();
  }
}

void ObstacleCameraPerception::SetCameraHeightAndPitch(
    const std::map<std::string, float> &name_camera_ground_height_map,
    const std::map<std::string, float> &name_camera_pitch_angle_diff_map,
    const float &pitch_angle_calibrator_working_sensor) {
  if (calibration_service_ == nullptr) {
    AERROR << "Calibraion service is not available";
    return;
  }
  calibration_service_->SetCameraHeightAndPitch(
      name_camera_ground_height_map, name_camera_pitch_angle_diff_map,
      pitch_angle_calibrator_working_sensor);
}

void ObstacleCameraPerception::SetIm2CarHomography(
    Eigen::Matrix3d homography_im2car) {
  if (calibration_service_ == nullptr) {
    AERROR << "Calibraion service is not available";
    return;
  }
  lane_postprocessor_->SetIm2CarHomography(homography_im2car);
}

bool ObstacleCameraPerception::GetCalibrationService(
    BaseCalibrationService **calibration_service) {
  *calibration_service = calibration_service_.get();
  return true;
}

bool ObstacleCameraPerception::Perception(
    const CameraPerceptionOptions &options, CameraFrame *frame) {
  PERCEPTION_PERF_FUNCTION();
  inference::CudaUtil::set_device_id(perception_param_.gpu_id());
  ObstacleDetectorOptions detector_options;
  ObstacleTransformerOptions transformer_options;
  ObstaclePostprocessorOptions obstacle_postprocessor_options;
  ObstacleTrackerOptions tracker_options;
  FeatureExtractorOptions extractor_options;
  PERCEPTION_PERF_BLOCK_START();
  frame->camera_k_matrix =
      name_intrinsic_map_.at(frame->data_provider->sensor_name());
  if (frame->calibration_service == nullptr) {
    AERROR << "Calibraion service is not available";
    return false;
  }

  LaneDetectorOptions lane_detetor_options;
  LanePostprocessorOptions lane_postprocessor_options;
  if (!lane_detector_->Detect(lane_detetor_options, frame)) {
    AERROR << "Failed to detect lane.";
    return false;
  }
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(frame->data_provider->sensor_name(),
                                           "LaneDetector");

  if (!lane_postprocessor_->Process2D(lane_postprocessor_options, frame)) {
    AERROR << "Failed to postprocess lane 2D.";
    return false;
  }
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(frame->data_provider->sensor_name(),
                                           "LanePostprocessor2D");

  // Calibration service
  frame->calibration_service->Update(frame);
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(frame->data_provider->sensor_name(),
                                           "CalibrationService");

  if (!lane_postprocessor_->Process3D(lane_postprocessor_options, frame)) {
    AERROR << "Failed to postprocess lane 3D.";
    return false;
  }
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(frame->data_provider->sensor_name(),
                                           "LanePostprocessor3D");

  if (write_out_lane_file_) {
    std::string lane_file_path =
        absl::StrCat(out_lane_dir_, "/", frame->frame_id, ".txt");
    WriteLanelines(write_out_lane_file_, lane_file_path, frame->lane_objects);
  }

  if (write_out_calib_file_) {
    std::string calib_file_path =
        absl::StrCat(out_calib_dir_, "/", frame->frame_id, ".txt");
    WriteCalibrationOutput(write_out_calib_file_, calib_file_path, frame);
  }

  // Obstacle prediction
  if (!tracker_->Predict(tracker_options, frame)) {
    AERROR << "Failed to predict.";
    return false;
  }
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(frame->data_provider->sensor_name(),
                                           "Predict");

  std::shared_ptr<BaseObstacleDetector> detector =
      name_detector_map_.at(frame->data_provider->sensor_name());

  if (!detector->Detect(detector_options, frame)) {
    AERROR << "Failed to detect.";
    return false;
  }
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(frame->data_provider->sensor_name(),
                                           "detect");

  // Save all detections results as kitti format
  WriteDetections(
      perception_param_.debug_param().has_detection_out_dir(),
      absl::StrCat(perception_param_.debug_param().detection_out_dir(), "/",
                   frame->frame_id, ".txt"),
      frame->detected_objects);
  if (extractor_ && !extractor_->Extract(extractor_options, frame)) {
    AERROR << "Failed to extractor";
    return false;
  }
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(frame->data_provider->sensor_name(),
                                           "external_feature");

  // Save detection results with bbox, detection_feature
  WriteDetections(
      perception_param_.debug_param().has_detect_feature_dir(),
      absl::StrCat(perception_param_.debug_param().detect_feature_dir(), "/",
                   frame->frame_id, ".txt"),
      frame);
  // Set the sensor name of each object
  for (size_t i = 0; i < frame->detected_objects.size(); ++i) {
    frame->detected_objects[i]->camera_supplement.sensor_name =
        frame->data_provider->sensor_name();
  }
  if (!tracker_->Associate2D(tracker_options, frame)) {
    AERROR << "Failed to associate2d.";
    return false;
  }
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(frame->data_provider->sensor_name(),
                                           "Associate2D");

  if (!transformer_->Transform(transformer_options, frame)) {
    AERROR << "Failed to transform.";
    return false;
  }
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(frame->data_provider->sensor_name(),
                                           "Transform");

  // Obstacle postprocessor
  obstacle_postprocessor_options.do_refinement_with_calibration_service =
      frame->calibration_service != nullptr;
  if (!obstacle_postprocessor_->Process(obstacle_postprocessor_options,
                                        frame)) {
    AERROR << "Failed to post process obstacles.";
    return false;
  }
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(frame->data_provider->sensor_name(),
                                           "PostprocessObsacle");

  if (!tracker_->Associate3D(tracker_options, frame)) {
    AERROR << "Failed to Associate3D.";
    return false;
  }
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(frame->data_provider->sensor_name(),
                                           "Associate3D");

  if (!tracker_->Track(tracker_options, frame)) {
    AERROR << "Failed to track.";
    return false;
  }
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(frame->data_provider->sensor_name(),
                                           "Track");

  if (perception_param_.has_debug_param()) {
    if (perception_param_.debug_param().has_camera2world_out_file()) {
      WriteCamera2World(out_pose_, frame->frame_id, frame->camera2world_pose);
    }
    if (perception_param_.debug_param().has_track_out_file()) {
      WriteTracking(out_track_, frame->frame_id, frame->tracked_objects);
    }
  }
  // Save tracked detections results as kitti format
  WriteDetections(
      perception_param_.debug_param().has_tracked_detection_out_dir(),
      absl::StrCat(perception_param_.debug_param().tracked_detection_out_dir(),
                   "/", frame->frame_id, ".txt"),
      frame->tracked_objects);

  // Fill polygon and set anchor point
  for (auto &obj : frame->tracked_objects) {
    FillObjectPolygonFromBBox3D(obj.get());
    obj->anchor_point = obj->center;
  }

  return true;
}
}  // namespace camera
}  // namespace perception
}  // namespace apollo
