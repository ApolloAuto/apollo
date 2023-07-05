/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/camera/app/obstacle_detection_camera.h"

#include <utility>

#include <boost/algorithm/string.hpp>

#include "absl/strings/str_cat.h"

#include "modules/perception/camera/lib/obstacle/detector/smoke/proto/smoke.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/util/perf_util.h"
#include "modules/perception/base/object.h"
#include "modules/perception/camera/app/debug_info.h"
#include "modules/perception/camera/common/global_config.h"
#include "modules/perception/camera/common/util.h"
#include "modules/perception/common/io/io_util.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/inference/utils/cuda_util.h"

namespace apollo {
namespace perception {
namespace camera {

using cyber::common::GetAbsolutePath;

bool ObstacleDetectionCamera::Init(const CameraPerceptionInitOptions &options) {
  std::string work_root;
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

bool ObstacleDetectionCamera::Init(const PipelineConfig &pipeline_config) {
  if (!Initialize(pipeline_config)) {
    return false;
  }

  camera_detection_config_ = pipeline_config.camera_detection_config();
  ACHECK(inference::CudaUtil::set_device_id(camera_detection_config_.gpu_id()));
  std::string work_root = GetCyberWorkRoot();
  // Init detector

  for (const auto stage_ptr : stage_ptrs_) {
    if (stage_ptr->stage_config_.type() == "camera_detector") {
      base::BaseCameraModelPtr model;
      auto stage_config = stage_ptr->stage_config_;
      std::string camera_name =
          stage_config.camera_detector_config().camera_name();
      boost::algorithm::split(camera_names_, camera_name,
                              boost::algorithm::is_any_of(","));

      for (size_t i = 0; i < camera_names_.size(); ++i) {
        model = common::SensorManager::Instance()->GetUndistortCameraModel(
            camera_names_[i]);
        auto pinhole = static_cast<base::PinholeCameraModel *>(model.get());
        name_intrinsic_map_.insert(std::pair<std::string, Eigen::Matrix3f>(
            camera_names_[i], pinhole->get_intrinsic_params()));
      }
    } else {
      continue;
    }
  }

  // Init tracker
  // Init transformer
  // Init obstacle postprocessor
  // Init debug_param
  if (camera_detection_config_.has_debug_param()) {
    // Init debug info
    if (camera_detection_config_.debug_param().has_track_out_file()) {
      out_track_.open(camera_detection_config_.debug_param().track_out_file(),
                      std::ofstream::out);
    }
    if (camera_detection_config_.debug_param().has_camera2world_out_file()) {
      out_pose_.open(
          camera_detection_config_.debug_param().camera2world_out_file(),
          std::ofstream::out);
    }
  }

  // Init object template
  if (camera_detection_config_.has_object_template_param()) {
    ObjectTemplateManagerInitOptions init_options;
    auto plugin_param =
        camera_detection_config_.object_template_param().plugin_param();
    // todo(zero): need fix work_root
    init_options.root_dir = GetAbsolutePath(work_root, plugin_param.root_dir());
    init_options.conf_file = plugin_param.config_file();
    ACHECK(ObjectTemplateManager::Instance()->Init(init_options));
  }

  return true;
}

bool ObstacleDetectionCamera::Process(DataFrame *data_frame) {
  CameraFrame *frame = data_frame->camera_frame;
  frame->camera_k_matrix =
      name_intrinsic_map_.at(frame->data_provider->sensor_name());
  for (size_t i = 1; i < camera_names_.size(); ++i) {
    if (camera_names_.size() == 2) {
      continue;
    } else {
      CameraFrame *frame = (data_frame + i)->camera_frame;
      frame->camera_k_matrix =
      name_intrinsic_map_.at(frame->data_provider->sensor_name());
    }
  }
  InnerProcess(data_frame);

  if (camera_detection_config_.has_debug_param()) {
    if (camera_detection_config_.debug_param().has_camera2world_out_file()) {
      WriteCamera2World(out_pose_, frame->frame_id, frame->camera2world_pose);
    }
    if (camera_detection_config_.debug_param().has_track_out_file()) {
      WriteTracking(out_track_, frame->frame_id, frame->tracked_objects);
    }
  }

  // Save tracked detections results as kitti format
  WriteDetections(
      camera_detection_config_.debug_param().has_tracked_detection_out_dir(),
      absl::StrCat(
          camera_detection_config_.debug_param().tracked_detection_out_dir(),
          "/", frame->frame_id, ".txt"),
      frame->tracked_objects);

  // Fill polygon and set anchor point
  for (auto &obj : frame->tracked_objects) {
    FillObjectPolygonFromBBox3D(obj.get());
    obj->anchor_point = obj->center;
  }
  return true;
}

bool ObstacleDetectionCamera::Perception(const CameraPerceptionOptions &options,
                                         CameraFrame *frame) {
  PERF_FUNCTION();
  inference::CudaUtil::set_device_id(perception_param_.gpu_id());
  ObstacleDetectorOptions detector_options;
  ObstacleTransformerOptions transformer_options;
  ObstaclePostprocessorOptions obstacle_postprocessor_options;
  ObstacleTrackerOptions tracker_options;
  FeatureExtractorOptions extractor_options;
  PERF_BLOCK_START();
  frame->camera_k_matrix =
      name_intrinsic_map_.at(frame->data_provider->sensor_name());

  // Obstacle prediction
  if (!tracker_->Predict(tracker_options, frame)) {
    AERROR << "Failed to predict.";
    return false;
  }
  PERF_BLOCK_END_WITH_INDICATOR(frame->data_provider->sensor_name(), "Predict");

  std::shared_ptr<BaseObstacleDetector> detector =
      name_detector_map_.at(frame->data_provider->sensor_name());

  if (!detector->Detect(detector_options, frame)) {
    AERROR << "Failed to detect.";
    return false;
  }
  PERF_BLOCK_END_WITH_INDICATOR(frame->data_provider->sensor_name(), "detect");

  // Save all detections results as kitti format
  WriteDetections(
      perception_param_.debug_param().has_detection_out_dir(),
      absl::StrCat(perception_param_.debug_param().detection_out_dir(), "/",
                   frame->frame_id, ".txt"),
      frame->detected_objects);
  // todo(zero): need fix
  // if (extractor_ && !extractor_->Extract(extractor_options, frame)) {
  //   AERROR << "Failed to extractor";
  //   return false;
  // }
  PERF_BLOCK_END_WITH_INDICATOR(frame->data_provider->sensor_name(),
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
  PERF_BLOCK_END_WITH_INDICATOR(frame->data_provider->sensor_name(),
                                "Associate2D");

  if (!transformer_->Transform(transformer_options, frame)) {
    AERROR << "Failed to transform.";
    return false;
  }
  PERF_BLOCK_END_WITH_INDICATOR(frame->data_provider->sensor_name(),
                                "Transform");

  // Obstacle postprocessor
  obstacle_postprocessor_options.do_refinement_with_calibration_service =
      frame->calibration_service != nullptr;
  if (!obstacle_postprocessor_->Process(obstacle_postprocessor_options,
                                        frame)) {
    AERROR << "Failed to post process obstacles.";
    return false;
  }
  PERF_BLOCK_END_WITH_INDICATOR(frame->data_provider->sensor_name(),
                                "PostprocessObsacle");

  if (!tracker_->Associate3D(tracker_options, frame)) {
    AERROR << "Failed to Associate3D.";
    return false;
  }
  PERF_BLOCK_END_WITH_INDICATOR(frame->data_provider->sensor_name(),
                                "Associate3D");

  if (!tracker_->Track(tracker_options, frame)) {
    AERROR << "Failed to track.";
    return false;
  }
  PERF_BLOCK_END_WITH_INDICATOR(frame->data_provider->sensor_name(), "Track");

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
