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
      ->GetValue("max_cached_image_lights_array_size", &_max_cached_image_lights_array_size)) {
    AERROR << "max_cached_image_lights_array_size not found." << Name();
    return false;
  }
  if (!model_config->GetValue("projection_image_cols", &_projection_image_cols)) {
    AERROR << "projection_image_cols not found." << Name();
    return false;
  }
  if (!model_config->GetValue("projection_image_rows", &_projection_image_rows)) {
    AERROR << "projection_image_rows not found." << Name();
    return false;
  }
  if (!model_config->GetValue("sync_interval_seconds", &_sync_interval_seconds)) {
    AERROR << "sync_interval_seconds not found." << Name();
    return false;
  }
  if (!model_config->GetValue("no_signals_interval_seconds", &_no_signals_interval_seconds)) {
    AERROR << "no_signals_interval_seconds not found." << Name();
    return false;
  }

  // init projection
  if (!_projection.init()) {
    AERROR << "TLPreprocessor init projection failed.";
    return false;
  }
  AINFO << kCountCameraId;
  return true;
}

bool TLPreprocessor::AddCachedLightsProjections(const CarPose &pose,
                                                const std::vector<apollo::hdmap::Signal> &signals,
                                                const double timestamp) {
  MutexLock lock(&_mutex);
  PERF_FUNCTION();

  AINFO << "TLPreprocessor has " << _cached_lights_projections_array.size()
        << " lights projections cached.";

  // pop front if cached array'size > FLAGS_max_cached_image_lights_array_size
  while (_cached_lights_projections_array.size() > _max_cached_image_lights_array_size) {
    _cached_lights_projections_array.erase(_cached_lights_projections_array.begin());
  }

  // lights projection info. to be added in cached array
  std::shared_ptr<ImageLights> image_lights(new ImageLights);
  image_lights->timestamp = timestamp;
  image_lights->pose = pose;
  image_lights->is_pose_valid = true;
  image_lights->camera_id = LONG_FOCUS;  // default select long focus camera

  // get light info from hd-map (need update _last_signals and _last_signals_ts out-scope)
  // ...
  AINFO << "TLPreprocessor Got signal number:" << signals.size()
        << ", ts: " << GLOG_TIMESTAMP(timestamp);
  for (int i = 0; i < signals.size(); ++i) {
    AINFO << "signal info:" << signals[i].ShortDebugString();
  }

  bool has_signals = true;
  if (signals.size() == 0) {
    has_signals = false;
    _last_no_signals_ts = timestamp;
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
  if (!has_signals) {
    image_lights->lights = lights_on_image_array[kLongFocusIdx];
    image_lights->lights_outside_image = lights_outside_image_array[kLongFocusIdx];
    _cached_lights_projections_array.push_back(image_lights);
    AINFO << "no signals, add lights projections to cached array.";
    return true;
  }

  // project light region on each camera's image plane
  for (int cam_id = 0; cam_id < kCountCameraId; ++cam_id) {
    if (!project_lights(signals,
                        pose,
                        static_cast<CameraId>(cam_id),
                        lights_on_image_array[cam_id],
                        lights_outside_image_array[cam_id])) {
      AERROR << "add_cached_lights_projections project lights on "
             << kCameraIdToStr.at(static_cast<CameraId>(cam_id)) << " image failed, "
             << "ts: " << GLOG_TIMESTAMP(timestamp)
             << ", camera_id: " << kCameraIdToStr.at(static_cast<CameraId>(cam_id));
      return false;
    }
  }

  // select which image to be used
  select_image(pose, lights_on_image_array, lights_outside_image_array, &(image_lights->camera_id));

  auto selected_camera_id = static_cast<int>(image_lights->camera_id);

  assert(selected_camera_id >= 0 && selected_camera_id < kCountCameraId);

  image_lights->lights = lights_on_image_array[selected_camera_id];
  image_lights->lights_outside_image = lights_outside_image_array[selected_camera_id];
  image_lights->num_signals = image_lights->lights->size() +
      image_lights->lights_outside_image->size();
  _cached_lights_projections_array.push_back(image_lights);
  AINFO << "selected_camera_id: " << kCameraIdToStr.at(selected_camera_id);

  return true;
}

bool TLPreprocessor::SyncImage(
    const ImageSharedPtr &image,
    const double ts,
    const CameraId &camera_i,
    std::shared_ptr<ImageLights> *image_lights,
    bool *should_pub) {
  MutexLock lock(&_mutex);
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
    auto camera_id_str = (kCameraIdToStr.find(camera_id) != kCameraIdToStr.end() ?
                          kCameraIdToStr.at(camera_id) : std::to_string(camera_id));
    AINFO << "sync_image_with_cached_lights_projections failed, "
          << "not publish image to shared data, ts: " << GLOG_TIMESTAMP(timestamp)
          << ", camera_id: " << camera_id_str;
    *should_pub = false;
    return false;
  }

  // 同步不成功，根据 camera_id 判断是否发送图像
  if (!sync_ok) {

    AINFO << "working camera with maximum focal length: "
          << kCameraIdToStr.at(kLongFocusIdx)
          << ", _last_pub_camera_id: " << _last_pub_camera_id;
    // 在缓存的 signals_num 中根据时间戳查找当前图像时间的灯数
    size_t current_signal_num = 0;

    // 如果灯数为 0 ，那么默认发焦距最大的相机的图像
    // 否则，判断上一次发的是不是也是最大焦距的图像
    if (camera_id == kLongFocusIdx &&
        (current_signal_num == 0 || camera_id == _last_pub_camera_id &&
            _last_pub_camera_id != CameraId::UNKNOWN)) {
      (*image_lights).reset(new ImageLights);
      (*image_lights)->image = image;
      // 距离查不到灯在一定时间范围以内，找不到 pose 是由于查 /tf 降频了，不做标记
      // 降低 debug 图像上 "No valid pose" 闪烁频率
      (*image_lights)->is_pose_valid =
          (fabs(timestamp - _last_no_signals_ts) < _no_signals_interval_seconds);
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
    _last_output_ts = timestamp;   // update last output timestamp only when sync. is really ok
  }
  _last_pub_camera_id = camera_id;
  *should_pub = true;

  return true;
}

void TLPreprocessor::set_last_pub_camera_id(CameraId camera_id) {
  _last_pub_camera_id = camera_id;
}

void TLPreprocessor::get_last_pub_camera_id(CameraId *camera_id) const {
  *camera_id = _last_pub_camera_id;
}

void TLPreprocessor::set_no_signals_interval_seconds(double seconds) {
  _no_signals_interval_seconds = seconds;
}

void TLPreprocessor::get_no_signals_interval_seconds(double *seconds) const {
  *seconds = _no_signals_interval_seconds;
}

bool TLPreprocessor::set_max_cached_image_lights_array_size(
    size_t max_cached_image_lights_array_size) {
  if (max_cached_image_lights_array_size <= 0) {
    AERROR << "max_cached_image_lights_array_size must be positive, input size: "
           << max_cached_image_lights_array_size;
    return false;
  }
  _max_cached_image_lights_array_size = max_cached_image_lights_array_size;

  return true;
}

bool TLPreprocessor::get_max_cached_image_lights_array_size(
    size_t *max_cached_image_lights_array_size) const {
  *max_cached_image_lights_array_size = _max_cached_image_lights_array_size;
  return true;
}

void TLPreprocessor::select_image(const CarPose &pose,
                                  const std::vector<std::shared_ptr<LightPtrs> > &lights_on_image_array,
                                  const std::vector<std::shared_ptr<LightPtrs> > &lights_outside_image_array,
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
        if (is_on_border(cv::Size(_projection_image_cols,
                                  _projection_image_rows),
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

bool TLPreprocessor::project_lights(const std::vector<apollo::hdmap::Signal> &signals,
                                    const CarPose &pose,
                                    CameraId camera_id,
                                    std::shared_ptr<LightPtrs> &lights_on_image,
                                    std::shared_ptr<LightPtrs> &lights_outside_image) {
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
    if (!_projection.project(pose, ProjectOption(camera_id), light.get())) {
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

  // find close enough(by timestamp difference) lights projection from back to front
  const auto &cached_array = _cached_lights_projections_array;
  *sync_ok = false;
  bool find_loc = false;  // 是否查到定位
  auto ptr_lights_projection = cached_array.rbegin();
  for (; ptr_lights_projection != cached_array.rend(); ++ptr_lights_projection) {
    double ts_proj = (*ptr_lights_projection)->timestamp;
    if (fabs(ts_proj - sync_time) < _sync_interval_seconds) {
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
        //return false;
        continue;
      }
      if (sync_time < _last_output_ts) {
        AWARN << "TLPreprocessor reject the image pub ts:"
              << GLOG_TIMESTAMP(sync_time)
              << " which is earlier than last output ts:"
              << GLOG_TIMESTAMP(_last_output_ts)
              << ", image camera_id: " << kCameraIdToStr.at(image_cam_id);
        return false;
      }
      *sync_ok = true;
      break;
    }
  }

  if (*sync_ok) {
    // elements from cached_array should not possess any pointer to image resource
    // so here we allocate new space and copy other contents
    *image_lights = *ptr_lights_projection;
//    image_lights->pose = (*ptr_lights_projection)->pose;
//    image_lights->camera_id = (*ptr_lights_projection)->camera_id;
//    image_lights->timestamp = sync_time;  // 图像时间戳
//    image_lights->is_pose_valid = (*ptr_lights_projection)->is_pose_valid;
//    image_lights->diff_image_pose_ts = (*ptr_lights_projection)->diff_image_pose_ts;
//    image_lights->lights = (*ptr_lights_projection)->lights;
//    image_lights->lights_outside_image = (*ptr_lights_projection)->lights_outside_image;
//    image_lights->num_signals = (*ptr_lights_projection)->num_signals;
//    image_lights->image = image;  // (*ptr_lights_projection)->image.get() == NULL
  }

  // 没找到对应时间的 pose，log 输出原因，diff_image_pose_ts 记录时间戳差值
  // 若 diff_image_pose_ts 为正，则当前图像时间戳比缓存中最晚的 pose 的时间戳还要晚
  // 若 diff_image_pose_ts 为负，则当前图像时间戳比缓存中最早的 pose 的时间戳还要早
  std::string cached_array_str = "_cached_lights_projections_array";

  if (!(*sync_ok) && cached_array.size() > 1) {
    // 由于没有灯时降低查 /tf 频率导致查不到定位，直接返回
    if (fabs(sync_time - _last_no_signals_ts) < _no_signals_interval_seconds) {
      AINFO << "TLPreprocessor " << cached_array_str << " sync failed, image ts: "
            << GLOG_TIMESTAMP(sync_time)
            << " last_no_signals_ts: " << GLOG_TIMESTAMP(_last_no_signals_ts)
            << " (sync_time - last_no_signals_ts): "
            << GLOG_TIMESTAMP(sync_time - _last_no_signals_ts)
            << " query /tf in low frequence. "
            << " camera_id: " << kCameraIdToStr.at(camera_id);
      return true;
    }
    if (sync_time < cached_array.front()->timestamp) {
      double pose_ts = cached_array.front()->timestamp;
      double system_ts = TimeUtil::GetCurrentTime();
      AWARN << "TLPreprocessor " << cached_array_str << " sync failed, image ts: "
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
      AWARN << "TLPreprocessor " << cached_array_str << " sync failed, image ts: "
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
      AWARN << "TLPreprocessor " << cached_array_str << " sync failed, image ts: "
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

bool TLPreprocessor::is_on_border(const cv::Size size,
                                  const cv::Rect &roi, const int border_size) const {
  if (roi.x < border_size || roi.y < border_size) {
    return true;
  }
  if (roi.x + roi.width + border_size >= size.width ||
      roi.y + roi.height + border_size >= size.height) {
    return true;
  }
  return false;
}

int TLPreprocessor::get_min_focal_len_camera_id() {

  return kShortFocusIdx;
}

int TLPreprocessor::get_max_focal_len_camera_id() {
  return kLongFocusIdx;
}

bool TLPreprocessor::select_camera_by_lights_projection(const double timestamp,
                                                        const CarPose &pose,
                                                        const std::vector<apollo::hdmap::Signal> &signals,
                                                        std::shared_ptr<ImageLights> *image_lights,
                                                        CameraId *selected_camera_id) {
  AINFO << "select_camera_by_lights_projection signals number: " << signals.size();

  bool has_signals = (signals.size() > 0);

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
                        lights_on_image_array[cam_id],
                        lights_outside_image_array[cam_id])) {
      AERROR << "select_camera_by_lights_projection project lights on "
             << kCameraIdToStr.at(cam_id) << " image failed, "
             << "ts: " << GLOG_TIMESTAMP(timestamp)
             << ", camera_id: " << kCameraIdToStr.at(cam_id);
      return false;
    }
  }

  // select which image to be used
  select_image(pose, lights_on_image_array, lights_outside_image_array, selected_camera_id);
  assert(*selected_camera_id >= 0 && *selected_camera_id < kCountCameraId);

  // set selected lights
  if (*selected_camera_id == (*image_lights)->camera_id) {
    (*image_lights)->lights = lights_on_image_array[*selected_camera_id];
    (*image_lights)->lights_outside_image = lights_outside_image_array[*selected_camera_id];
    (*image_lights)->num_signals =
        (*image_lights)->lights->size() + (*image_lights)->lights_outside_image->size();
  }

  return true;
}

REGISTER_PREPROCESSOR(TLPreprocessor);

} // namespace traffic_light
} // namespace perception
} // namespace apollo
