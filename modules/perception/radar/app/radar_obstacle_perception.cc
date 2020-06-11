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
#include "modules/perception/radar/app/radar_obstacle_perception.h"

#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lib/registerer/registerer.h"
#include "modules/perception/lib/utils/perf.h"

using apollo::perception::lib::ConfigManager;
using apollo::perception::lib::ModelConfig;

namespace apollo {
namespace perception {
namespace radar {

bool RadarObstaclePerception::Init(const std::string& pipeline_name) {
  std::string model_name = pipeline_name;
  const ModelConfig* model_config = nullptr;
  ACHECK(ConfigManager::Instance()->GetModelConfig(model_name, &model_config))
      << "not found model: " << model_name;

  std::string detector_name;
  ACHECK(model_config->get_value("Detector", &detector_name))
      << "Detector not found";

  std::string roi_filter_name;
  ACHECK(model_config->get_value("RoiFilter", &roi_filter_name))
      << "RoiFilter not found";

  std::string tracker_name;
  ACHECK(model_config->get_value("Tracker", &tracker_name))
      << "Tracker not found";

  BaseDetector* detector =
      BaseDetectorRegisterer::GetInstanceByName(detector_name);
  CHECK_NOTNULL(detector);
  detector_.reset(detector);

  BaseRoiFilter* roi_filter =
      BaseRoiFilterRegisterer::GetInstanceByName(roi_filter_name);
  CHECK_NOTNULL(roi_filter);
  roi_filter_.reset(roi_filter);

  BaseTracker* tracker = BaseTrackerRegisterer::GetInstanceByName(tracker_name);
  CHECK_NOTNULL(tracker);
  tracker_.reset(tracker);

  ACHECK(detector_->Init()) << "radar detector init error";
  ACHECK(roi_filter_->Init()) << "radar roi filter init error";
  ACHECK(tracker_->Init()) << "radar tracker init error";

  return true;
}

bool RadarObstaclePerception::Perceive(
    const drivers::ContiRadar& corrected_obstacles,
    const RadarPerceptionOptions& options,
    std::vector<base::ObjectPtr>* objects) {
  PERCEPTION_PERF_FUNCTION();
  const std::string& sensor_name = options.sensor_name;
  PERCEPTION_PERF_BLOCK_START();
  base::FramePtr detect_frame_ptr(new base::Frame());

  if (!detector_->Detect(corrected_obstacles, options.detector_options,
                         detect_frame_ptr)) {
    AERROR << "radar detect error";
    return false;
  }
  ADEBUG << "Detected frame objects number: "
         << detect_frame_ptr->objects.size();
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(sensor_name, "detector");
  if (!roi_filter_->RoiFilter(options.roi_filter_options, detect_frame_ptr)) {
    ADEBUG << "All radar objects were filtered out";
  }
  ADEBUG << "RoiFiltered frame objects number: "
         << detect_frame_ptr->objects.size();
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(sensor_name, "roi_filter");

  base::FramePtr tracker_frame_ptr = std::make_shared<base::Frame>();
  if (!tracker_->Track(*detect_frame_ptr, options.track_options,
                       tracker_frame_ptr)) {
    AERROR << "radar track error";
    return false;
  }
  ADEBUG << "tracked frame objects number: "
         << tracker_frame_ptr->objects.size();
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(sensor_name, "tracker");

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
