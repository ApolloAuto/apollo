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
#include "modules/perception/lidar/app/lidar_obstacle_tracking.h"

#include "cyber/common/file.h"
#include "modules/common/util/perf_util.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/common/lidar_log.h"

#include "modules/perception/lidar/app/proto/lidar_obstacle_tracking_config.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

bool LidarObstacleTracking::Init(
    const LidarObstacleTrackingInitOptions& options) {
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
                                               "lidar_obstacle_tracking.conf");

  LidarObstacleTrackingConfig config;
  ACHECK(cyber::common::GetProtoFromFile(config_file, &config));

  BaseMultiTargetTracker* multi_target_tracker =
      BaseMultiTargetTrackerRegisterer::
      GetInstanceByName(config.multi_target_tracker());
  CHECK_NOTNULL(multi_target_tracker);
  multi_target_tracker_.reset(multi_target_tracker);
  MultiTargetTrackerInitOptions tracker_init_options;
  ACHECK(multi_target_tracker_->Init(tracker_init_options)) <<
                              "lidar multi_target_tracker init error";

  BaseClassifier* fusion_classifier =
      BaseClassifierRegisterer::GetInstanceByName(config.fusion_classifier());
  CHECK_NOTNULL(fusion_classifier);
  fusion_classifier_.reset(fusion_classifier);
  ClassifierInitOptions fusion_classifier_init_options;
  ACHECK(fusion_classifier_->Init(fusion_classifier_init_options)) <<
                              "lidar classifier init error";
  return true;
}

bool LidarObstacleTracking::Init(const PipelineConfig& pipeline_config) {
  return Initialize(pipeline_config);
}

bool LidarObstacleTracking::Process(DataFrame* data_frame) {
  if (data_frame == nullptr)
    return false;

  return InnerProcess(data_frame);
}

LidarProcessResult LidarObstacleTracking::Process(
    const LidarObstacleTrackingOptions& options, LidarFrame* frame) {
  const auto& sensor_name = options.sensor_name;

  PERF_FUNCTION_WITH_INDICATOR(sensor_name);

  PERF_BLOCK_START();
  MultiTargetTrackerOptions tracker_options;
  if (!multi_target_tracker_->Track(tracker_options, frame)) {
    return LidarProcessResult(LidarErrorCode::TrackerError,
                              "Fail to track objects.");
  }
  PERF_BLOCK_END_WITH_INDICATOR(sensor_name, "tracker");

  ClassifierOptions fusion_classifier_options;
  if (!fusion_classifier_->Classify(fusion_classifier_options, frame)) {
    return LidarProcessResult(LidarErrorCode::ClassifierError,
                              "Fail to fuse object types.");
  }
  PERF_BLOCK_END_WITH_INDICATOR(sensor_name, "type_fusion");

  return LidarProcessResult(LidarErrorCode::Succeed);
}

PERCEPTION_REGISTER_LIDAROBSTACLETRACKING(LidarObstacleTracking);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
