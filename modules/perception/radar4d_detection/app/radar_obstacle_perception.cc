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
#include "modules/perception/radar4d_detection/app/radar_obstacle_perception.h"

#include "modules/perception/radar4d_detection/proto/radar_obstacle_perception_config.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/util/perf_util.h"
#include "modules/perception/common/lib/registerer/registerer.h"
#include "modules/perception/common/util.h"

namespace apollo {
namespace perception {
namespace radar4d {

bool RadarObstaclePerception::Init(const PerceptionInitOptions& options) {
  // Init config
  std::string config_file =
      GetConfigFile(options.config_path, options.config_file);
  RadarObstaclePerceptionConfig radar_obstacle_config;
  if (!cyber::common::GetProtoFromFile(config_file, &radar_obstacle_config)) {
    AERROR << "Read config failed: " << config_file;
    return false;
  }

  // object detector
  auto detector_param = radar_obstacle_config.detector_param();
  DetectorInitOptions detector_init_options;
  detector_init_options.config_path = detector_param.config_path();
  detector_init_options.config_file = detector_param.config_file();
  BaseDetector* detector =
      BaseDetectorRegisterer::GetInstanceByName(detector_param.name());
  CHECK_NOTNULL(detector);
  detector_.reset(detector);
  ACHECK(detector_->Init(detector_init_options)) << "radar detector init error";

  // object builder
  enable_roi_filter_ = radar_obstacle_config.enable_roi_filter();
  ACHECK(builder_.Init(detector_init_options)) << "radar builder init error";

  // hdmap roi filter
  auto roi_filter_param = radar_obstacle_config.roi_filter_param();
  RoiFilterInitOptions roi_filter_init_options;
  roi_filter_init_options.config_path = roi_filter_param.config_path();
  roi_filter_init_options.config_file = roi_filter_param.config_file();
  BaseRoiFilter* roi_filter =
      BaseRoiFilterRegisterer::GetInstanceByName(roi_filter_param.name());
  CHECK_NOTNULL(roi_filter);
  roi_filter_.reset(roi_filter);
  ACHECK(roi_filter_->Init(roi_filter_init_options))
      << "radar roi filter init error";

  // multi target tracking init
  auto multi_target_tracker_param =
    radar_obstacle_config.multi_target_tracker_param();
  multi_target_tracker_ = BaseMultiTargetTrackerRegisterer::GetInstanceByName(
      multi_target_tracker_param.name());
  CHECK_NOTNULL(multi_target_tracker_);

  MultiTargetTrackerInitOptions tracker_init_options;
  tracker_init_options.config_path = multi_target_tracker_param.config_path();
  tracker_init_options.config_file = multi_target_tracker_param.config_file();
  ACHECK(multi_target_tracker_->Init(tracker_init_options));

  // fused classifier init
  auto fusion_classifier_param =
    radar_obstacle_config.fusion_classifier_param();
  fusion_classifier_ = BaseClassifierRegisterer::GetInstanceByName(
        fusion_classifier_param.name());
  CHECK_NOTNULL(fusion_classifier_);

  ClassifierInitOptions fusion_classifier_init_options;
  fusion_classifier_init_options.config_path =
      fusion_classifier_param.config_path();
  fusion_classifier_init_options.config_file =
      fusion_classifier_param.config_file();
  ACHECK(fusion_classifier_->Init(fusion_classifier_init_options));
  return true;
}

bool RadarObstaclePerception::Perceive(
    RadarFrame* frame,
    const RadarPerceptionOptions& options,
    std::vector<base::ObjectPtr>* objects) {
  PERF_FUNCTION();
  const std::string& sensor_name = options.sensor_name;
  PERF_BLOCK_START();

  // object detection
  if (!detector_->Detect(frame, options.detector_options)) {
    AERROR << "radar detect error";
    return false;
  }
  // object builder
  if (!builder_.Build(options.detector_options, frame)) {
    AERROR << "radar detector, object builder error.";
    return false;
  }
  AINFO << "Detected frame objects number: "
         << frame->segmented_objects.size();
  PERF_BLOCK_END_WITH_INDICATOR(sensor_name, "detector");

  // roi filter
  if (enable_roi_filter_) {
    if (!roi_filter_->RoiFilter(options.roi_filter_options, frame)) {
      AINFO << "All radar objects were filtered out";
    }
    AINFO << "RoiFiltered frame objects number: "
          << frame->segmented_objects.size();
    PERF_BLOCK_END_WITH_INDICATOR(sensor_name, "roi_filter");
  }

  // multi target tracker
  MultiTargetTrackerOptions tracker_options;
  if (!multi_target_tracker_->Track(tracker_options, frame)) {
    AERROR << "radar4d tracking, multi_target_tracker_ Track error.";
    return false;
  }
  PERF_BLOCK_END_WITH_INDICATOR(sensor_name, "tracker");

  // fused classifer
  // ClassifierOptions fusion_classifier_options;
  // if (!fusion_classifier_->Classify(fusion_classifier_options,
  //                                   frame)) {
  //   AERROR << "radar4d tracking, fusion_classifier_ Classify error.";
  //   return false;
  // }
  // PERF_BLOCK_END_WITH_INDICATOR(sensor_name, "tracker");

  *objects = frame->tracked_objects;

  return true;
}

std::string RadarObstaclePerception::Name() const {
  return "RadarObstaclePerception";
}

PERCEPTION_REGISTER_RADAR_OBSTACLE_PERCEPTION(RadarObstaclePerception);

}  // namespace radar4d
}  // namespace perception
}  // namespace apollo
