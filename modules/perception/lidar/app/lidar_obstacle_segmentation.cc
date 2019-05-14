/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/lidar/app/lidar_obstacle_segmentation.h"

#include "cyber/common/file.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lib/utils/perf.h"
#include "modules/perception/lidar/app/proto/lidar_obstacle_segmentation_config.pb.h"
#include "modules/perception/lidar/common/lidar_log.h"
#include "modules/perception/lidar/lib/scene_manager/scene_manager.h"

namespace apollo {
namespace perception {
namespace lidar {

bool LidarObstacleSegmentation::Init(
    const LidarObstacleSegmentationInitOptions& options) {
  auto& sensor_name = options.sensor_name;
  auto config_manager = lib::ConfigManager::Instance();
  const lib::ModelConfig* model_config = nullptr;
  CHECK(config_manager->GetModelConfig(Name(), &model_config));

  const std::string work_root = config_manager->work_root();
  std::string config_file;
  std::string root_path;
  CHECK(model_config->get_value("root_path", &root_path));
  config_file = cyber::common::GetAbsolutePath(work_root, root_path);
  config_file = cyber::common::GetAbsolutePath(config_file, sensor_name);
  config_file = cyber::common::GetAbsolutePath(
      config_file, "lidar_obstacle_segmentation.conf");

  LidarObstacleSegmentationConfig config;
  CHECK(cyber::common::GetProtoFromFile(config_file, &config));
  segmentor_name_ = config.segmentor();
  use_map_manager_ = config.use_map_manager();
  use_object_filter_bank_ = config.use_object_filter_bank();

  use_map_manager_ = use_map_manager_ && options.enable_hdmap_input;

  SceneManagerInitOptions scene_manager_init_options;
  CHECK(SceneManager::Instance().Init(scene_manager_init_options));

  PointCloudPreprocessorInitOptions preprocessor_init_options;
  preprocessor_init_options.sensor_name = sensor_name;
  CHECK(cloud_preprocessor_.Init(preprocessor_init_options));

  if (use_map_manager_) {
    MapManagerInitOptions map_manager_init_options;
    if (!map_manager_.Init(map_manager_init_options)) {
      AINFO << "Failed to init map manager.";
      use_map_manager_ = false;
    }
  }

  segmentor_.reset(
      BaseSegmentationRegisterer::GetInstanceByName(segmentor_name_));
  CHECK_NOTNULL(segmentor_.get());
  SegmentationInitOptions segmentation_init_options;
  segmentation_init_options.sensor_name = sensor_name;
  CHECK(segmentor_->Init(segmentation_init_options));

  ObjectBuilderInitOptions builder_init_options;
  CHECK(builder_.Init(builder_init_options));

  if (use_object_filter_bank_) {
    ObjectFilterInitOptions filter_bank_init_options;
    filter_bank_init_options.sensor_name = sensor_name;
    CHECK(filter_bank_.Init(filter_bank_init_options));
  }

  return true;
}

LidarProcessResult LidarObstacleSegmentation::Process(
    const LidarObstacleSegmentationOptions& options, LidarFrame* frame) {
  PointCloudPreprocessorOptions preprocessor_options;
  preprocessor_options.sensor2novatel_extrinsics =
      options.sensor2novatel_extrinsics;
  if (cloud_preprocessor_.Preprocess(preprocessor_options, frame)) {
    return ProcessCommon(options, frame);
  }
  return LidarProcessResult(LidarErrorCode::PointCloudPreprocessorError,
                            "Failed to preprocess point cloud.");
}

LidarProcessResult LidarObstacleSegmentation::Process(
    const LidarObstacleSegmentationOptions& options,
    const std::shared_ptr<apollo::drivers::PointCloud const>& message,
    LidarFrame* frame) {
  const auto& sensor_name = options.sensor_name;

  PERCEPTION_PERF_FUNCTION_WITH_INDICATOR(options.sensor_name);

  PERCEPTION_PERF_BLOCK_START();
  PointCloudPreprocessorOptions preprocessor_options;
  preprocessor_options.sensor2novatel_extrinsics =
      options.sensor2novatel_extrinsics;
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(sensor_name, "preprocess");
  if (cloud_preprocessor_.Preprocess(preprocessor_options, message, frame)) {
    return ProcessCommon(options, frame);
  }
  return LidarProcessResult(LidarErrorCode::PointCloudPreprocessorError,
                            "Failed to preprocess point cloud.");
}

LidarProcessResult LidarObstacleSegmentation::ProcessCommon(
    const LidarObstacleSegmentationOptions& options, LidarFrame* frame) {
  const auto& sensor_name = options.sensor_name;

  PERCEPTION_PERF_BLOCK_START();
  if (use_map_manager_) {
    MapManagerOptions map_manager_options;
    if (!map_manager_.Update(map_manager_options, frame)) {
      return LidarProcessResult(LidarErrorCode::MapManagerError,
                                "Failed to update map structure.");
    }
  }
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(sensor_name, "map_manager");

  SegmentationOptions segmentation_options;
  if (!segmentor_->Segment(segmentation_options, frame)) {
    return LidarProcessResult(LidarErrorCode::SegmentationError,
                              "Failed to segment.");
  }
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(sensor_name, "segmentation");

  ObjectBuilderOptions builder_options;
  if (!builder_.Build(builder_options, frame)) {
    return LidarProcessResult(LidarErrorCode::ObjectBuilderError,
                              "Failed to build objects.");
  }
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(sensor_name, "object_builder");

  ObjectFilterOptions filter_options;
  if (!filter_bank_.Filter(filter_options, frame)) {
    return LidarProcessResult(LidarErrorCode::ObjectFilterError,
                              "Failed to filter objects.");
  }
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(sensor_name, "filter_bank");

  return LidarProcessResult(LidarErrorCode::Succeed);
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
