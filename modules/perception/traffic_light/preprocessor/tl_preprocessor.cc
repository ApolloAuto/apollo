/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/traffic_light/preprocessor/tl_preprocessor.h"

#include "modules/common/time/time_util.h"
#include "modules/perception/onboard/transform_input.h"
#include "modules/perception/traffic_light/base/tl_shared_data.h"
#include "modules/perception/traffic_light/base/utils.h"

namespace apollo {
namespace perception {
namespace traffic_light {

using apollo::common::time::TimeUtil;

bool TLPreprocessor::Init() {
  ConfigManager *config_manager = ConfigManager::instance();
  const ModelConfig *model_config = config_manager->GetModelConfig(name());
  if (model_config == nullptr) {
    AERROR << "not found model: " << name();
    return false;
  }

  // Read parameters from config file
  if (!model_config->GetValue("max_cached_lights_size",
                              &max_cached_lights_size_)) {
    AERROR << "max_cached_image_lights_array_size not found." << name();
    return false;
  }
  if (!model_config->GetValue("projection_image_cols",
                              &projection_image_cols_)) {
    AERROR << "projection_image_cols not found." << name();
    return false;
  }
  if (!model_config->GetValue("projection_image_rows",
                              &projection_image_rows_)) {
    AERROR << "projection_image_rows not found." << name();
    return false;
  }
  if (!model_config->GetValue("sync_interval_seconds",
                              &sync_interval_seconds_)) {
    AERROR << "sync_interval_seconds not found." << name();
    return false;
  }
  if (!model_config->GetValue("no_signals_interval_seconds",
                              &no_signals_interval_seconds_)) {
    AERROR << "no_signals_interval_seconds not found." << name();
    return false;
  }

  // init projection
  if (!projection_.Init()) {
    AERROR << "TLPreprocessor init projection failed.";
    return false;
  }
  AINFO << kCountCameraId;
  return true;
}

bool TLPreprocessor::CacheLightsProjections(const CarPose &pose,
                                            const std::vector<Signal> &signals,
                                            const double timestamp) {
  MutexLock lock(&mutex_);
  PERF_FUNCTION();

  AINFO << "TLPreprocessor has " << cached_lights_.size()
        << " lights projections cached.";

  // pop front if cached array'size > FLAGS_max_cached_image_lights_array_size
  while (cached_lights_.size() > static_cast<size_t>(max_cached_lights_size_)) {
    cached_lights_.erase(cached_lights_.begin());
  }

  // lights projection info. to be added in cached array
  std::shared_ptr<ImageLights> image_lights(new ImageLights);
  // default select long focus camera
  image_lights->camera_id = LONG_FOCUS;
  image_lights->timestamp = timestamp;
  image_lights->pose = pose;
  image_lights->is_pose_valid = true;

  AINFO << "TLPreprocessor Got signal number:" << signals.size()
        << ", ts: " << GLOG_TIMESTAMP(timestamp);
  for (const auto &signal : signals) {
    AINFO << "signal info:" << signal.ShortDebugString();
  }
  // lights projections info.

  std::vector<std::shared_ptr<LightPtrs>> lights_on_image(kCountCameraId);
  std::vector<std::shared_ptr<LightPtrs>> lights_outside_image(kCountCameraId);
  for (auto &light_ptrs : lights_on_image) {
    light_ptrs.reset(new LightPtrs);
  }
  for (auto &light_ptrs : lights_outside_image) {
    light_ptrs.reset(new LightPtrs);
  }
  if (signals.size() > 0) {
    // project light region on each camera's image plane
    for (int cam_id = 0; cam_id < kCountCameraId; ++cam_id) {
      if (!ProjectLights(pose, signals, static_cast<CameraId>(cam_id),
                         lights_on_image[cam_id].get(),
                         lights_outside_image[cam_id].get())) {
        AERROR << "add_cached_lights_projections project lights on "
               << kCameraIdToStr.at(static_cast<CameraId>(cam_id))
               << " image failed, "
               << "ts: " << GLOG_TIMESTAMP(timestamp) << ", camera_id: "
               << kCameraIdToStr.at(static_cast<CameraId>(cam_id));
        return false;
      }
    }

    // select which image to be used
    SelectImage(pose, lights_on_image, lights_outside_image,
                &(image_lights->camera_id));
    AINFO << "select camera: " << kCameraIdToStr.at(image_lights->camera_id);

  } else {
    last_no_signals_ts_ = timestamp;
  }
  image_lights->num_signals = signals.size();
  AINFO << "cached info with " << image_lights->num_signals << " signals";
  cached_lights_.push_back(image_lights);

  return true;
}

bool TLPreprocessor::SyncImage(const ImageSharedPtr &image,
                               ImageLightsPtr *image_lights, bool *should_pub) {
  MutexLock lock(&mutex_);
  PERF_FUNCTION();
  CameraId camera_id = image->camera_id();
  double image_ts = image->ts();
  bool sync_ok = false;

  PERF_FUNCTION();
  if (cached_lights_.size() == 0) {
    AINFO << "No cached light";
    return false;
  }
  const int cam_id = static_cast<int>(camera_id);
  if (cam_id < 0 || cam_id >= kCountCameraId) {
    AERROR << "SyncImage failed, "
           << "get unknown CameraId: " << camera_id;
    return false;
  }

  // find close enough(by timestamp difference)
  // lights projection from back to front

  bool find_loc = false;  // if pose is found
  auto cached_lights_ptr = cached_lights_.rbegin();
  for (; cached_lights_ptr != cached_lights_.rend(); ++cached_lights_ptr) {
    double light_ts = (*cached_lights_ptr)->timestamp;
    if (fabs(light_ts - image_ts) < sync_interval_seconds_) {
      find_loc = true;
      auto proj_cam_id = static_cast<int>((*cached_lights_ptr)->camera_id);
      auto image_cam_id = static_cast<int>(camera_id);
      auto proj_cam_id_str =
          (kCameraIdToStr.find(proj_cam_id) != kCameraIdToStr.end()
               ? kCameraIdToStr.at(proj_cam_id)
               : std::to_string(proj_cam_id));
      // found related pose but if camear ID doesn't match
      if (proj_cam_id != image_cam_id) {
        AWARN << "find appropriate localization, but camera_id not match"
              << ", cached projection's camera_id: " << proj_cam_id_str
              << " , image's camera_id: " << kCameraIdToStr.at(image_cam_id);
        continue;
      }
      if (image_ts < last_output_ts_) {
        AWARN << "TLPreprocessor reject the image pub ts:"
              << GLOG_TIMESTAMP(image_ts)
              << " which is earlier than last output ts:"
              << GLOG_TIMESTAMP(last_output_ts_)
              << ", image camera_id: " << kCameraIdToStr.at(image_cam_id);
        return false;
      }
      sync_ok = true;
      break;
    }
  }

  if (sync_ok) {
    *image_lights = *cached_lights_ptr;
    (*image_lights)->diff_image_pose_ts =
        image_ts - (*cached_lights_ptr)->timestamp;
    (*image_lights)->diff_image_sys_ts = image_ts - TimeUtil::GetCurrentTime();

    (*image_lights)->image = image;
    (*image_lights)->timestamp = image_ts;
    AINFO << "TLPreprocessor sync ok ts: " << GLOG_TIMESTAMP(image_ts)
          << " camera_id: " << kCameraIdToStr.at(camera_id);
    last_output_ts_ = image_ts;
    last_pub_camera_id_ = camera_id;
    *should_pub = true;
  } else {
    AINFO << "sync image with cached lights projection failed, "
          << "no valid pose, ts: " << GLOG_TIMESTAMP(image_ts)
          << " camera_id: " << kCameraIdToStr.at(camera_id);
    std::string cached_array_str = "cached lights";
    double diff_image_pose_ts = 0.0;
    double diff_image_sys_ts = 0.0;
    bool no_signal = false;
    if (fabs(image_ts - last_no_signals_ts_) < no_signals_interval_seconds_) {
      AINFO << "TLPreprocessor " << cached_array_str
            << " sync failed, image ts: " << GLOG_TIMESTAMP(image_ts)
            << " last_no_signals_ts: " << GLOG_TIMESTAMP(last_no_signals_ts_)
            << " (sync_time - last_no_signals_ts): "
            << GLOG_TIMESTAMP(image_ts - last_no_signals_ts_)
            << " query /tf in low frequence because no signals forward "
            << " camera_id: " << kCameraIdToStr.at(camera_id);
      no_signal = true;
    } else if (image_ts < cached_lights_.front()->timestamp) {
      double pose_ts = cached_lights_.front()->timestamp;
      double system_ts = TimeUtil::GetCurrentTime();
      AWARN << "TLPreprocessor " << cached_array_str
            << " sync failed, image ts: " << GLOG_TIMESTAMP(image_ts)
            << ", which is earlier than " << cached_array_str
            << ".front() ts: " << GLOG_TIMESTAMP(pose_ts)
            << ", diff between image and pose ts: "
            << GLOG_TIMESTAMP(image_ts - pose_ts)
            << "; system ts: " << GLOG_TIMESTAMP(system_ts)
            << ", diff between image and system ts: "
            << GLOG_TIMESTAMP(image_ts - system_ts)
            << ", camera_id: " << kCameraIdToStr.at(camera_id);
      // difference between image and pose timestamps
      diff_image_pose_ts = image_ts - pose_ts;
      diff_image_sys_ts = image_ts - system_ts;
    } else if (image_ts > cached_lights_.back()->timestamp) {
      double pose_ts = cached_lights_.back()->timestamp;
      double system_ts = TimeUtil::GetCurrentTime();
      AWARN << "TLPreprocessor " << cached_array_str
            << " sync failed, image ts: " << GLOG_TIMESTAMP(image_ts)
            << ", which is older than " << cached_array_str
            << ".back() ts: " << GLOG_TIMESTAMP(pose_ts)
            << ", diff between image and pose ts: "
            << GLOG_TIMESTAMP(image_ts - pose_ts)
            << "; system ts: " << GLOG_TIMESTAMP(system_ts)
            << ", diff between image and system ts: "
            << GLOG_TIMESTAMP(image_ts - system_ts)
            << ", camera_id: " << kCameraIdToStr.at(camera_id);
      diff_image_pose_ts = image_ts - pose_ts;
      diff_image_sys_ts = image_ts - system_ts;
    } else if (!find_loc) {
      // if no pose found, log warning msg
      AWARN << "TLPreprocessor " << cached_array_str
            << " sync failed, image ts: " << GLOG_TIMESTAMP(image_ts)
            << ", cannot find close enough timestamp, " << cached_array_str
            << ".front() ts: "
            << GLOG_TIMESTAMP(cached_lights_.front()->timestamp) << ", "
            << cached_array_str << ".back() ts: "
            << GLOG_TIMESTAMP(cached_lights_.back()->timestamp)
            << ", camera_id: " << kCameraIdToStr.at(camera_id);
    }
    if (image->camera_id() == LONG_FOCUS &&
        (no_signal || last_pub_camera_id_ == LONG_FOCUS)) {
      *should_pub = true;
      (*image_lights).reset(new ImageLights);
      (*image_lights)->image = image;
      (*image_lights)->camera_id = image->camera_id();
      (*image_lights)->timestamp = image_ts;
      (*image_lights)->diff_image_sys_ts = diff_image_sys_ts;
      (*image_lights)->diff_image_pose_ts = diff_image_pose_ts;
      (*image_lights)->is_pose_valid = no_signal;
      (*image_lights)->num_signals = 0;
    }
  }
  // sync fail may because:
  // 1. image is not selected
  // 2. timestamp drift
  // 3. [there is no tf]
  return sync_ok;
}

void TLPreprocessor::set_last_pub_camera_id(CameraId camera_id) {
  last_pub_camera_id_ = camera_id;
}

CameraId TLPreprocessor::last_pub_camera_id() const {
  return last_pub_camera_id_;
}

int TLPreprocessor::max_cached_lights_size() const {
  return max_cached_lights_size_;
}

void TLPreprocessor::SelectImage(const CarPose &pose,
                                 const LightsArray &lights_on_image_array,
                                 const LightsArray &lights_outside_image_array,
                                 CameraId *selection) {
  *selection = static_cast<CameraId>(kShortFocusIdx);

  // check from long focus to short focus
  for (int cam_id = 0; cam_id < kCountCameraId; ++cam_id) {
    if (!lights_outside_image_array[cam_id]->empty()) {
      continue;
    }
    bool ok = true;
    // find the short focus camera without range check
    if (cam_id != kShortFocusIdx) {
      for (const LightPtr &light : *(lights_on_image_array[cam_id])) {
        if (IsOnBorder(cv::Size(projection_image_cols_, projection_image_rows_),
                       light->region.projection_roi,
                       image_border_size[cam_id])) {
          ok = false;
          AINFO << "light project on image border region, "
                << "CameraId: " << kCameraIdToStr.at(cam_id);
          break;
        }
      }
    }
    if (ok) {
      *selection = static_cast<CameraId>(cam_id);
      break;
    }
  }

  AINFO << "select_image selection: " << *selection;
}

bool TLPreprocessor::ProjectLights(const CarPose &pose,
                                   const std::vector<Signal> &signals,
                                   const CameraId &camera_id,
                                   LightPtrs *lights_on_image,
                                   LightPtrs *lights_outside_image) {
  if (signals.empty()) {
    ADEBUG << "project_lights get empty signals.";
    return true;
  }

  const int cam_id = static_cast<int>(camera_id);
  if (cam_id < 0 || cam_id >= kCountCameraId) {
    AERROR << "project_lights get invalid CameraId: " << camera_id;
    return false;
  }

  for (size_t i = 0; i < signals.size(); ++i) {
    LightPtr light(new Light);
    light->info = signals[i];
    if (!projection_.Project(pose, ProjectOption(camera_id), light.get())) {
      lights_outside_image->push_back(light);
    } else {
      lights_on_image->push_back(light);
    }
  }

  return true;
}

bool TLPreprocessor::IsOnBorder(const cv::Size size, const cv::Rect &roi,
                                const int border_size) const {
  if (roi.x < border_size || roi.y < border_size) {
    return true;
  }
  if (roi.x + roi.width + border_size >= size.width ||
      roi.y + roi.height + border_size >= size.height) {
    return true;
  }
  return false;
}

int TLPreprocessor::GetMinFocalLenCameraId() { return kShortFocusIdx; }

int TLPreprocessor::GetMaxFocalLenCameraId() { return kLongFocusIdx; }

REGISTER_PREPROCESSOR(TLPreprocessor);

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
