/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/lidar/app/lidar_obstacle_detection.h"

#include "cyber/common/file.h"
#include "modules/common/util/perf_util.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/app/proto/lidar_obstacle_detection_config.pb.h"
#include "modules/perception/lidar/common/lidar_log.h"

namespace apollo {
namespace perception {
namespace lidar {

bool LidarObstacleDetection::Init(
    const LidarObstacleDetectionInitOptions& options) {
  auto& sensor_name = options.sensor_name;
  auto config_manager = lib::ConfigManager::Instance();
  const lib::ModelConfig* model_config = nullptr;
  ACHECK(config_manager->GetModelConfig(Name(), &model_config));

  const std::string work_root = config_manager->work_root();
  std::string config_file;
  std::string root_path;
  ACHECK(model_config->get_value("root_path", &root_path));
  config_file = cyber::common::GetAbsolutePath(work_root, root_path);
  config_file = cyber::common::GetAbsolutePath(config_file, sensor_name);
  config_file = cyber::common::GetAbsolutePath(config_file,
                                               "lidar_obstacle_detection.conf");

  LidarObstacleDetectionConfig config;
  ACHECK(cyber::common::GetProtoFromFile(config_file, &config));
  detector_name_ = config.detector();

  PointCloudPreprocessorInitOptions preprocessor_init_options;
  preprocessor_init_options.sensor_name = sensor_name;
  ACHECK(cloud_preprocessor_.Init(preprocessor_init_options));

  detector_.reset(new PointPillarsDetection);
  // detector_.reset(
  //    BaseSegmentationRegisterer::GetInstanceByName(segmentor_name_));
  CHECK_NOTNULL(detector_.get());
  DetectionInitOptions detection_init_options;
  // segmentation_init_options.sensor_name = sensor_name;
  ACHECK(detector_->Init(detection_init_options));

  return true;
}

LidarProcessResult LidarObstacleDetection::Process(
    const LidarObstacleDetectionOptions& options, LidarFrame* frame) {
  PointCloudPreprocessorOptions preprocessor_options;
  preprocessor_options.sensor2novatel_extrinsics =
      options.sensor2novatel_extrinsics;
  if (cloud_preprocessor_.Preprocess(preprocessor_options, frame)) {
    return ProcessCommon(options, frame);
  }
  return LidarProcessResult(LidarErrorCode::PointCloudPreprocessorError,
                            "Failed to preprocess point cloud.");
}

LidarProcessResult LidarObstacleDetection::Process(
    const LidarObstacleDetectionOptions& options,
    const std::shared_ptr<apollo::drivers::PointCloud const>& message,
    LidarFrame* frame) {
  const auto& sensor_name = options.sensor_name;

  PERF_FUNCTION_WITH_INDICATOR(options.sensor_name);

  PERF_BLOCK_START();
  PointCloudPreprocessorOptions preprocessor_options;
  preprocessor_options.sensor2novatel_extrinsics =
      options.sensor2novatel_extrinsics;
  PERF_BLOCK_END_WITH_INDICATOR(sensor_name, "preprocess");
  if (cloud_preprocessor_.Preprocess(preprocessor_options, message, frame)) {
    return ProcessCommon(options, frame);
  }
  return LidarProcessResult(LidarErrorCode::PointCloudPreprocessorError,
                            "Failed to preprocess point cloud.");
}

LidarProcessResult LidarObstacleDetection::ProcessCommon(
    const LidarObstacleDetectionOptions& options, LidarFrame* frame) {
  const auto& sensor_name = options.sensor_name;

  PERF_BLOCK_START();
  DetectionOptions detection_options;
  if (!detector_->Detect(detection_options, frame)) {
    return LidarProcessResult(LidarErrorCode::DetectionError,
                              "Failed to detect.");
  }
  PERF_BLOCK_END_WITH_INDICATOR(sensor_name, "detection");

  return LidarProcessResult(LidarErrorCode::Succeed);
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
