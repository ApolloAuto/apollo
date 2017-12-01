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
#include "modules/perception/lib/base/time_util.h"
#include "modules/perception/traffic_light/base/utils.h"
#include "modules/perception/onboard/transform_input.h"
#include "modules/perception/traffic_light/base/tl_shared_data.h"
namespace apollo {
namespace perception {
namespace traffic_light {

bool TLPreprocessor::Init() {
  ConfigManager *config_manager = ConfigManager::instance();
  const ModelConfig *model_config = NULL;
  if (!config_manager->GetModelConfig(Name(), &model_config)) {
    AERROR << "not found model: " << Name();
    return false;
  }

  // Read parameters from config file
  if (!model_config
      ->GetValue("max_cached_lights_size",
                 &max_cached_lights_size_)) {
    AERROR << "max_cached_image_lights_array_size not found." << Name();
    return false;
  }
  if (!model_config->GetValue("projection_image_cols",
                              &projection_image_cols_)) {
    AERROR << "projection_image_cols not found." << Name();
    return false;
  }
  if (!model_config->GetValue("projection_image_rows",
                              &projection_image_rows_)) {
    AERROR << "projection_image_rows not found." << Name();
    return false;
  }
  if (!model_config->GetValue("sync_interval_seconds",
                              &sync_interval_seconds_)) {
    AERROR << "sync_interval_seconds not found." << Name();
    return false;
  }
  if (!model_config->GetValue("no_signals_interval_seconds",
                              &no_signals_interval_seconds_)) {
    AERROR << "no_signals_interval_seconds not found." << Name();
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
  while (cached_lights_.size() > max_cached_lights_size_) {
    cached_lights_.erase(cached_lights_.begin());
  }

  // lights projection info. to be added in cached array
  std::shared_ptr<ImageLights> image_lights(new ImageLights);
  image_lights->timestamp = timestamp;
  image_lights->pose = pose;
  image_lights->is_pose_valid = true;
  // default select long focus camera
  image_lights->camera_id = LONG_FOCUS;

  AINFO << "TLPreprocessor Got signal number:" << signals.size()
        << ", ts: " << GLOG_TIMESTAMP(timestamp);
  for (int i = 0; i < signals.size(); ++i) {
    AINFO << "signal info:" << signals[i].ShortDebugString();
  }
  // lights projections info.

  std::vector<std::shared_ptr<LightPtrs> > lights_on_image_array(
      kCountCameraId);
  std::vector<std::shared_ptr<LightPtrs> > lights_outside_image_array(
      kCountCameraId);
  for (auto &light_ptrs : lights_on_image_array) {
    light_ptrs.reset(new LightPtrs);
  }
  for (auto &light_ptrs : lights_outside_image_array) {
    light_ptrs.reset(new LightPtrs);
  }

  // 当前查不到灯，标记一下没有灯，添加到缓存队列
  if (signals.size() == 0) {
    last_no_signals_ts_ = timestamp;
    image_lights->lights = lights_on_image_array[kLongFocusIdx];
    image_lights->lights_outside_image =
        lights_outside_image_array[kLongFocusIdx];
    cached_lights_.push_back(image_lights);
    AINFO << "no signals, add lights projections to cached array.";
    return true;
  }

  // project light region on each camera's image plane
  for (int cam_id = 0; cam_id < kCountCameraId; ++cam_id) {
    if (!project_lights(signals,
                        pose,
                        static_cast<CameraId>(cam_id),
                        lights_on_image_array[cam_id].get(),
                        lights_outside_image_array[cam_id].get())) {
      AERROR << "add_cached_lights_projections project lights on "
             << kCameraIdToStr.at(static_cast<CameraId>(cam_id))
             << " image failed, "
             << "ts: " << GLOG_TIMESTAMP(timestamp)
             << ", camera_id: "
             << kCameraIdToStr.at(static_cast<CameraId>(cam_id));
      return false;
    }
  }

  // select which image to be used
  SelectImage(pose,
              lights_on_image_array,
              lights_outside_image_array,
              &(image_lights->camera_id));

  auto selected_camera_id = static_cast<int>(image_lights->camera_id);

  assert(selected_camera_id >= 0 && selected_camera_id < kCountCameraId);

  image_lights->lights = lights_on_image_array[selected_camera_id];
  image_lights->lights_outside_image =
      lights_outside_image_array[selected_camera_id];
  image_lights->num_signals = image_lights->lights->size() +
      image_lights->lights_outside_image->size();
  cached_lights_.push_back(image_lights);
  AINFO << "selected_camera_id: " << kCameraIdToStr.at(selected_camera_id);

  return true;
}

bool TLPreprocessor::SyncImage(
    const ImageSharedPtr &image,
    const double ts,
    const CameraId &camera_i,
    std::shared_ptr<ImageLights> *image_lights,
    bool *should_pub) {
  MutexLock lock(&mutex_);
  PERF_FUNCTION();
  CameraId camera_id = image->device_id();
  double timestamp = image->ts();
  bool sync_ok = false;
  double diff_image_pose_ts = 0.0;
  double diff_image_sys_ts = 0.0;

  // sync image with cached lights projections
  if (!sync_image_with_cached_lights_projections(
      image, camera_id, timestamp, image_lights,
      &diff_image_pose_ts, &diff_image_sys_ts, &sync_ok)) {
    auto camera_id_str =
        (kCameraIdToStr.find(camera_id) != kCameraIdToStr.end() ?
         kCameraIdToStr.at(camera_id) : std::to_string(camera_id));
    AINFO << "sync_image_with_cached_lights_projections failed, "
          << "not publish image to shared data, ts: "
          << GLOG_TIMESTAMP(timestamp)
          << ", camera_id: " << camera_id_str;
    *should_pub = false;
    return false;
  }

  // 同步不成功，根据 camera_id 判断是否发送图像
  if (!sync_ok) {
    AINFO << "working camera with maximum focal length: "
          << kCameraIdToStr.at(kLongFocusIdx)
          << ", _last_pub_camera_id: " << last_pub_camera_id_;
    // 在缓存的 signals_num 中根据时间戳查找当前图像时间的灯数
    size_t current_signal_num = 0;

    // 如果灯数为 0 ，那么默认发焦距最大的相机的图像
    // 否则，判断上一次发的是不是也是最大焦距的图像
    if (camera_id == kLongFocusIdx &&
        (current_signal_num == 0 || camera_id == last_pub_camera_id_ &&
            last_pub_camera_id_ != CameraId::UNKNOWN)) {
      (*image_lights).reset(new ImageLights);
      (*image_lights)->image = image;
      // 距离查不到灯在一定时间范围以内，
      // 找不到 pose 是由于查 /tf 降频了，不做标记
      // 降低 debug 图像上 "No valid pose" 闪烁频率
      (*image_lights)->is_pose_valid =
          (fabs(timestamp - last_no_signals_ts_)
              < no_signals_interval_seconds_);
      (*image_lights)->diff_image_pose_ts = diff_image_pose_ts;
      (*image_lights)->diff_image_sys_ts = diff_image_sys_ts;
      (*image_lights)->timestamp = timestamp;
      (*image_lights)->camera_id = camera_id;
      (*image_lights)->lights.reset(new LightPtrs);
      (*image_lights)->lights_outside_image.reset(new LightPtrs);
      // 使用查找到的灯数
      (*image_lights)->num_signals = current_signal_num;

      AINFO << "sync image with cached lights projection failed, "
            << "no valid pose, ts: " << GLOG_TIMESTAMP(timestamp)
            << " camera_id: " << kCameraIdToStr.at(camera_id);

    } else {  // 其他 camera_id，直接返回，不发送图像
      AINFO << "sync image with cached lights projection failed, "
            << "no valid pose, ts: " << GLOG_TIMESTAMP(timestamp)
            << " camera_id: " << kCameraIdToStr.at(camera_id);
      *should_pub = false;
      return false;
    }
  }
  if (sync_ok) {
    AINFO << "TLPreprocessor sync ok ts: " << GLOG_TIMESTAMP(timestamp)
          << " camera_id: " << kCameraIdToStr.at(camera_id);
    last_output_ts_ = timestamp;
  }
  last_pub_camera_id_ = camera_id;
  *should_pub = true;

  return true;
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
  // 找当前工作的焦距最小的相机，不进行边界检查

  for (size_t cam_id = 0; cam_id < lights_on_image_array.size(); ++cam_id) {
    if (!lights_outside_image_array[cam_id]->empty()) {
      continue;
    }
    bool ok = true;
    if (cam_id != kShortFocusIdx) {
      for (const LightPtr &light : *(lights_on_image_array[cam_id])) {
        if (IsOnBorder(cv::Size(projection_image_cols_,
                                projection_image_rows_),
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

bool TLPreprocessor::
project_lights(const std::vector<Signal> &signals,
               const CarPose &pose,
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

bool TLPreprocessor::sync_image_with_cached_lights_projections(
    const ImageSharedPtr &image,
    CameraId camera_id,
    double sync_time,
    std::shared_ptr<ImageLights> *image_lights,
    double *diff_image_pose_ts,
    double *diff_image_sys_ts,
    bool *sync_ok) {
  PERF_FUNCTION();
  const int cam_id = static_cast<int>(camera_id);
  if (cam_id < 0 || cam_id >= kCountCameraId) {
    AERROR << "sync_image_with_cached_lights_projections failed, "
           << "get unknown CameraId: " << camera_id;
    return false;
  }

  // find close enough(by timestamp difference)
  // lights projection from back to front
  const auto &cached_array = cached_lights_;
  *sync_ok = false;
  bool find_loc = false;  // 是否查到定位
  auto ptr_lights_projection = cached_array.rbegin();
  for (; ptr_lights_projection != cached_array.rend();
         ++ptr_lights_projection) {
    double ts_proj = (*ptr_lights_projection)->timestamp;
    if (fabs(ts_proj - sync_time) < sync_interval_seconds_) {
      find_loc = true;
      auto proj_cam_id = static_cast<int>((*ptr_lights_projection)->camera_id);
      auto image_cam_id = static_cast<int>(camera_id);
      auto proj_cam_id_str =
          (kCameraIdToStr.find(proj_cam_id) != kCameraIdToStr.end() ?
           kCameraIdToStr.at(proj_cam_id) : std::to_string(proj_cam_id));
      // 找到对应时间的定位，但是相机 ID 不符
      if (camera_id != (*ptr_lights_projection)->camera_id) {
        AWARN << "find appropriate localization, but camera_id not match"
              << ", cached projection's camera_id: "
              << proj_cam_id_str
              << " , image's camera_id: "
              << kCameraIdToStr.at(image_cam_id);
        continue;
      }
      if (sync_time < last_output_ts_) {
        AWARN << "TLPreprocessor reject the image pub ts:"
              << GLOG_TIMESTAMP(sync_time)
              << " which is earlier than last output ts:"
              << GLOG_TIMESTAMP(last_output_ts_)
              << ", image camera_id: " << kCameraIdToStr.at(image_cam_id);
        return false;
      }
      *sync_ok = true;
      break;
    }
  }

  if (*sync_ok) {
    *image_lights = *ptr_lights_projection;
  }

  std::string cached_array_str = "_cached_lights_projections_array";

  if (!(*sync_ok) && cached_array.size() > 1) {
    if (fabs(sync_time - last_no_signals_ts_) < no_signals_interval_seconds_) {
      AINFO << "TLPreprocessor " << cached_array_str
            << " sync failed, image ts: "
            << GLOG_TIMESTAMP(sync_time)
            << " last_no_signals_ts: " << GLOG_TIMESTAMP(last_no_signals_ts_)
            << " (sync_time - last_no_signals_ts): "
            << GLOG_TIMESTAMP(sync_time - last_no_signals_ts_)
            << " query /tf in low frequence. "
            << " camera_id: " << kCameraIdToStr.at(camera_id);
      return true;
    }
    if (sync_time < cached_array.front()->timestamp) {
      double pose_ts = cached_array.front()->timestamp;
      double system_ts = TimeUtil::GetCurrentTime();
      AWARN << "TLPreprocessor " << cached_array_str
            << " sync failed, image ts: "
            << GLOG_TIMESTAMP(sync_time)
            << ", which is earlier than " << cached_array_str << ".front() ts: "
            << GLOG_TIMESTAMP(pose_ts)
            << ", diff between image and pose ts: "
            << GLOG_TIMESTAMP(sync_time - pose_ts)
            << "; system ts: " << GLOG_TIMESTAMP(system_ts)
            << ", diff between image and system ts: "
            << GLOG_TIMESTAMP(sync_time - system_ts)
            << ", camera_id: " << kCameraIdToStr.at(camera_id);
      // difference between image and pose timestamps
      *diff_image_pose_ts = sync_time - pose_ts;
      *diff_image_sys_ts = sync_time - system_ts;
    } else if (sync_time > cached_array.back()->timestamp) {
      double pose_ts = cached_array.back()->timestamp;
      double system_ts = TimeUtil::GetCurrentTime();
      AWARN << "TLPreprocessor " << cached_array_str
            << " sync failed, image ts: "
            << GLOG_TIMESTAMP(sync_time)
            << ", which is older than " << cached_array_str << ".back() ts: "
            << GLOG_TIMESTAMP(pose_ts)
            << ", diff between image and pose ts: "
            << GLOG_TIMESTAMP(sync_time - pose_ts)
            << "; system ts: " << GLOG_TIMESTAMP(system_ts)
            << ", diff between image and system ts: "
            << GLOG_TIMESTAMP(sync_time - system_ts)
            << ", camera_id: " << kCameraIdToStr.at(camera_id);
      *diff_image_pose_ts = sync_time - pose_ts;
      *diff_image_sys_ts = sync_time - system_ts;
    } else if (!find_loc) {
      // 确实没找到定位才打 log
      AWARN << "TLPreprocessor " << cached_array_str
            << " sync failed, image ts: "
            << GLOG_TIMESTAMP(sync_time)
            << ", cannot find close enough timestamp, "
            << cached_array_str << ".front() ts: "
            << GLOG_TIMESTAMP(cached_array.front()->timestamp) << ", "
            << cached_array_str << ".back() ts: "
            << GLOG_TIMESTAMP(cached_array.back()->timestamp)
            << ", camera_id: " << kCameraIdToStr.at(camera_id);
    }
  }

  return true;
}

bool TLPreprocessor::IsOnBorder(const cv::Size size,
                                const cv::Rect &roi,
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

int TLPreprocessor::GetMinFocalLenCameraId() {
  return kShortFocusIdx;
}

int TLPreprocessor::GetMaxFocalLenCameraId() {
  return kLongFocusIdx;
}

bool TLPreprocessor::
SelectCameraByProjection(const double timestamp,
                         const CarPose &pose,
                         const std::vector<Signal> &signals,
                         std::shared_ptr<ImageLights> *image_lights,
                         CameraId *selected_camera_id) {
  AINFO << "select_camera_by_lights_projection signals number: "
        << signals.size();

  // lights projections info.
  std::vector<std::shared_ptr<LightPtrs> > lights_on_image_array(
      kCountCameraId);
  std::vector<std::shared_ptr<LightPtrs> > lights_outside_image_array(
      kCountCameraId);
  for (auto &light_ptrs : lights_on_image_array) {
    light_ptrs.reset(new LightPtrs);
  }
  for (auto &light_ptrs : lights_outside_image_array) {
    light_ptrs.reset(new LightPtrs);
  }

  // project light region on each camera's image plane
  for (int cam_id = 0; cam_id < kCountCameraId; ++cam_id) {
    if (!project_lights(signals,
                        pose,
                        static_cast<CameraId>(cam_id),
                        lights_on_image_array[cam_id].get(),
                        lights_outside_image_array[cam_id].get())) {
      AERROR << "select_camera_by_lights_projection project lights on "
             << kCameraIdToStr.at(cam_id) << " image failed, "
             << "ts: " << GLOG_TIMESTAMP(timestamp)
             << ", camera_id: " << kCameraIdToStr.at(cam_id);
      return false;
    }
  }

  // select which image to be used
  SelectImage(pose,
              lights_on_image_array,
              lights_outside_image_array,
              selected_camera_id);
  assert(*selected_camera_id >= 0 && *selected_camera_id < kCountCameraId);

  // set selected lights
  if (*selected_camera_id == (*image_lights)->camera_id) {
    (*image_lights)->lights = lights_on_image_array[*selected_camera_id];
    (*image_lights)->lights_outside_image =
        lights_outside_image_array[*selected_camera_id];
    (*image_lights)->num_signals =
        (*image_lights)->lights->size()
            + (*image_lights)->lights_outside_image->size();
  }

  return true;
}

REGISTER_PREPROCESSOR(TLPreprocessor);

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
