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
#include "modules/perception/radar_detection/app/radar_obstacle_perception.h"

#include "modules/perception/radar_detection/proto/radar_obstacle_perception.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/util/perf_util.h"
#include "modules/perception/common/lib/registerer/registerer.h"
#include "modules/perception/common/util.h"

namespace apollo {
namespace perception {
namespace radar {

bool RadarObstaclePerception::Init(const PerceptionInitOptions& options) {
  std::string config_file =
      GetConfigFile(options.config_path, options.config_file);
  RadarObstaclePerceptionConfig radar_obstacle_config;
  if (!cyber::common::GetProtoFromFile(config_file, &radar_obstacle_config)) {
    AERROR << "Read config failed: " << config_file;
    return false;
  }

  auto detector_param = radar_obstacle_config.detector_param();
  DetectorInitOptions detector_init_options;
  detector_init_options.config_path = detector_param.config_path();
  detector_init_options.config_file = detector_param.config_file();
  BaseDetector* detector =
      BaseDetectorRegisterer::GetInstanceByName(detector_param.name());
  CHECK_NOTNULL(detector);
  detector_.reset(detector);
  ACHECK(detector_->Init(detector_init_options)) << "radar detector init error";

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

  auto tracker_param = radar_obstacle_config.tracker_param();
  TrackerInitOptions tracker_init_options;
  tracker_init_options.config_path = tracker_param.config_path();
  tracker_init_options.config_file = tracker_param.config_file();
  BaseTracker* tracker =
      BaseTrackerRegisterer::GetInstanceByName(tracker_param.name());
  CHECK_NOTNULL(tracker);
  tracker_.reset(tracker);
  ACHECK(tracker_->Init(tracker_init_options)) << "radar tracker init error";

  return true;
}

bool RadarObstaclePerception::Perceive(
    const drivers::ContiRadar& corrected_obstacles,
    const RadarPerceptionOptions& options,
    std::vector<base::ObjectPtr>* objects) {
  PERF_FUNCTION();
  const std::string& sensor_name = options.sensor_name;
  PERF_BLOCK_START();
  base::FramePtr detect_frame_ptr(new base::Frame());

  if (!detector_->Detect(corrected_obstacles, options.detector_options,
                         detect_frame_ptr)) {
    AERROR << "radar detect error";
    return false;
  }
  ADEBUG << "Detected frame objects number: "
         << detect_frame_ptr->objects.size();
  PERF_BLOCK_END_WITH_INDICATOR(sensor_name, "detector");
  if (!roi_filter_->RoiFilter(options.roi_filter_options, detect_frame_ptr)) {
    ADEBUG << "All radar objects were filtered out";
  }
  ADEBUG << "RoiFiltered frame objects number: "
         << detect_frame_ptr->objects.size();
  PERF_BLOCK_END_WITH_INDICATOR(sensor_name, "roi_filter");

  base::FramePtr tracker_frame_ptr(new base::Frame);
  if (!tracker_->Track(*detect_frame_ptr, options.track_options,
                       tracker_frame_ptr)) {
    AERROR << "radar track error";
    return false;
  }
  ADEBUG << "tracked frame objects number: "
         << tracker_frame_ptr->objects.size();
  PERF_BLOCK_END_WITH_INDICATOR(sensor_name, "tracker");

  *objects = tracker_frame_ptr->objects;

  return true;
}

std::string RadarObstaclePerception::Name() const {
  return "RadarObstaclePerception";
}

PERCEPTION_REGISTER_RADAR_OBSTACLE_PERCEPTION(RadarObstaclePerception);

}  // namespace radar
}  // namespace perception
}  // namespace apollo
