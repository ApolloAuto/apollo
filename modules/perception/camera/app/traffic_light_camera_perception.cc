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
#include "modules/perception/camera/app/traffic_light_camera_perception.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/camera/common/util.h"
#include "modules/perception/camera/lib/traffic_light/detector/detection/detection.h"
#include "modules/perception/camera/lib/traffic_light/detector/recognition/recognition.h"
#include "modules/perception/camera/lib/traffic_light/tracker/semantic_decision.h"
#include "modules/perception/lib/utils/perf.h"

namespace apollo {
namespace perception {
namespace camera {

using cyber::common::GetAbsolutePath;

bool TrafficLightCameraPerception::Init(
    const CameraPerceptionInitOptions &options) {
  std::string work_root = "";
  if (options.use_cyber_work_root) {
    work_root = GetCyberWorkRoot();
  }
  std::string proto_path = GetAbsolutePath(options.root_dir, options.conf_file);
  proto_path = GetAbsolutePath(work_root, proto_path);
  AINFO << "proto_path " << proto_path;
  if (!cyber::common::GetProtoFromFile(proto_path, &tl_param_)) {
    AINFO << "load proto param failed, root dir: " << options.root_dir;
    return false;
  }

  TrafficLightDetectorInitOptions init_options;
  auto plugin_param = tl_param_.detector_param(0).plugin_param();

  init_options.root_dir = GetAbsolutePath(work_root, plugin_param.root_dir());
  init_options.conf_file = plugin_param.config_file();
  init_options.gpu_id = tl_param_.gpu_id();
  detector_.reset(BaseTrafficLightDetectorRegisterer::GetInstanceByName(
      plugin_param.name()));
  CHECK(detector_ != nullptr);
  if (!detector_->Init(init_options)) {
    AERROR << "tl detector init failed";
    return false;
  }

  plugin_param = tl_param_.detector_param(1).plugin_param();
  init_options.root_dir = GetAbsolutePath(work_root, plugin_param.root_dir());
  init_options.conf_file = plugin_param.config_file();
  init_options.gpu_id = tl_param_.gpu_id();
  recognizer_.reset(BaseTrafficLightDetectorRegisterer::GetInstanceByName(
      plugin_param.name()));
  CHECK(recognizer_ != nullptr);
  if (!recognizer_->Init(init_options)) {
    AERROR << "tl recognizer init failed";
    return false;
  }

  TrafficLightTrackerInitOptions tracker_init_options;
  auto tracker_plugin_param = tl_param_.tracker_param().plugin_param();
  tracker_init_options.root_dir =
      GetAbsolutePath(work_root, tracker_plugin_param.root_dir());
  tracker_init_options.conf_file = tracker_plugin_param.config_file();
  tracker_.reset(BaseTrafficLightTrackerRegisterer::GetInstanceByName(
      tracker_plugin_param.name()));
  CHECK(tracker_ != nullptr);
  AINFO << tracker_init_options.root_dir << " "
        << tracker_init_options.conf_file;
  if (!tracker_->Init(tracker_init_options)) {
    AERROR << "tl tracker init failed";
    return false;
  }

  AINFO << "tl pipeline init done";
  return true;
}

bool TrafficLightCameraPerception::Perception(
    const CameraPerceptionOptions &options, CameraFrame *frame) {
  PERCEPTION_PERF_FUNCTION();
  PERCEPTION_PERF_BLOCK_START();
  TrafficLightDetectorOptions detector_options;
  if (!detector_->Detect(detector_options, frame)) {
    AERROR << "tl failed to detect.";
    return false;
  }
  const auto traffic_light_detect_time =
      PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(
          frame->data_provider->sensor_name(), "traffic_light_detect");

  TrafficLightDetectorOptions recognizer_options;
  if (!recognizer_->Detect(recognizer_options, frame)) {
    AERROR << "tl failed to recognize.";
    return false;
  }
  const auto traffic_light_recognize_time =
      PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(
          frame->data_provider->sensor_name(), "traffic_light_recognize");

  TrafficLightTrackerOptions tracker_options;
  if (!tracker_->Track(tracker_options, frame)) {
    AERROR << "tl failed to track.";
    return false;
  }
  const auto traffic_light_track_time =
      PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(
          frame->data_provider->sensor_name(), "traffic_light_track");
  AINFO << "TrafficLightsPerception perf_info."
        << " number_of_lights: " << frame->traffic_lights.size()
        << " traffic_light_detect_time: " << traffic_light_detect_time << " ms."
        << " traffic_light_recognize_time: " << traffic_light_recognize_time
        << " ms."
        << " traffic_light_track_time: " << traffic_light_track_time << " ms.";
  return true;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
