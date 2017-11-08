// Copyright (c) 2017 Baidu.com, Inc. All Rights Reserved
// @author guiyilin(guiyilin@baidu.com)
// @date 2017/08/07
// @file: tl_preprocessor.cpp
// @brief: tl_preprocessor definition
//

#include "module/perception/traffic_light/preprocessor/tl_preprocessor.h"

#include <gflags/gflags.h>
#include "xlog.h"

#include "lib/base/macros.h"
#include "lib/base/file_util.h"
#include "lib/base/time_util.h"
#include "lib/config_manager/config_manager.h"
#include <traffic_light/base/utils.h>

namespace adu {
namespace perception {
namespace traffic_light {

bool TLPreprocessor::init() {
  config_manager::ConfigManager *config_manager
      = base::Singleton<config_manager::ConfigManager>::get();
  const config_manager::ModelConfig *model_config = NULL;
  if (!config_manager->get_model_config(name(), &model_config)) {
    AERROR << "not found model: " << name();
    return false;
  }

  using config_manager::ConfigRead;

  // Read parameters from config file
  try {
    _max_cached_image_lights_array_size = ConfigRead<int>::read(*model_config,
                                                                "max_cached_image_lights_array_size");
    _projection_image_cols = ConfigRead<int>::read(*model_config,
                                                   "projection_image_cols");
    _projection_image_rows = ConfigRead<int>::read(*model_config,
                                                   "projection_image_rows");
    _sync_interval_seconds = ConfigRead<double>::read(*model_config,
                                                      "sync_interval_seconds");
    _no_signals_interval_seconds = ConfigRead<double>::read(*model_config,
                                                            "no_signals_interval_seconds");
  } catch (const config_manager::ConfigManagerError &e) {
    AERROR << name() << " failed to load configuration: " << e.what();
    return false;
  }

  return true;
}

bool TLPreprocessor::add_cached_lights_projections(
    const CarPose &pose,
    const std::vector<adu::common::hdmap::Signal> &signals,
    const MultiCamerasProjection &projection,
    const std::map<int, int> &image_borders_size,
    const double timestamp,
    bool *projections_outside_all_images) {
  base::MutexLock lock(&_mutex);
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
  image_lights->has_signals = true;
  image_lights->camera_id = LONG_FOCUS;  // default select long focus camera

  // get car pose
  // ...
  // get light info from hd-map (need update _last_signals and _last_signals_ts out-scope)
  // ...
  AINFO << "TLPreprocessor Got signal number:" << signals.size()
        << ", ts: " << GLOG_TIMESTAMP(timestamp);
  for (int i = 0; i < signals.size(); ++i) {
    AINFO << "signal info:" << signals[i].ShortDebugString();
  }

  // cache signal number
  _cached_signal_nums_array.push_back(std::make_pair(timestamp, signals.size()));
  while (_cached_signal_nums_array.size() > _max_cached_signal_nums_size) {
    _cached_signal_nums_array.erase(_cached_signal_nums_array.begin());
  }

  bool has_signals = true;
  if (signals.size() == 0) {
    has_signals = false;
    _last_no_signals_ts = timestamp;
  }

  // lights projections info.
  const int num_camera_ids = static_cast<int>(CameraId::CAMERA_ID_COUNT) - 1;
  const int long_focus_idx = static_cast<int>(CameraId::LONG_FOCUS);
  const int short_focus_idx = static_cast<int>(CameraId::SHORT_FOCUS);
  const int wide_focus_idx = static_cast<int>(CameraId::WIDE_FOCUS);
  const int narrow_focus_idx = static_cast<int>(CameraId::NARROW_FOCUS);

  std::vector<std::shared_ptr<LightPtrs> > lights_on_image_array(
      num_camera_ids);
  std::vector<std::shared_ptr<LightPtrs> > lights_outside_image_array(
      num_camera_ids);
  for (auto &light_ptrs : lights_on_image_array) {
    light_ptrs.reset(new LightPtrs);
  }
  for (auto &light_ptrs : lights_outside_image_array) {
    light_ptrs.reset(new LightPtrs);
  }

  // 当前查不到灯，标记一下没有灯，添加到缓存队列
  if (!has_signals) {
    image_lights->lights = lights_on_image_array[long_focus_idx];
    image_lights->lights_outside_image = lights_outside_image_array[long_focus_idx];
    image_lights->has_signals = false;
    _cached_lights_projections_array.push_back(image_lights);
    AINFO << "no signals, add lights projections to cached array.";
    return true;
  }

  // project light region on each camera's image plane
  for (int cam_id = 0; cam_id < num_camera_ids; ++cam_id) {
    if (!project_lights(projection, signals, pose,
                        static_cast<CameraId>(cam_id),
                        lights_on_image_array[cam_id],
                        lights_outside_image_array[cam_id])) {
      AERROR << "add_cached_lights_projections project lights on "
             << CAMERA_ID_TO_STR.at(cam_id) << " image failed, "
             << "ts: " << GLOG_TIMESTAMP(timestamp)
             << ", camera_id: " << CAMERA_ID_TO_STR.at(cam_id);
      return false;
    }
  }

  *projections_outside_all_images = has_signals;
  // 有灯 && (lights_on_image_array 里一个灯都没有), 说明所有相机的图像里都没有灯投影
  for (int cam_id = 0; cam_id < num_camera_ids; ++cam_id) {
    *projections_outside_all_images = *projections_outside_all_images &&
        (lights_on_image_array[cam_id]->size() == 0);
  }

  // select which image to be used
  select_image(pose,
               lights_on_image_array,
               lights_outside_image_array,
               image_borders_size,
               &(image_lights->camera_id));

  auto selected_camera_id = static_cast<int>(image_lights->camera_id);
  // if (selected_camera_id < 0 || selected_camera_id >= num_camera_ids) {
  //     AINFO << "Unknown selected_camera_id: " << selected_camera_id;
  //     return false;
  // }
  assert(selected_camera_id >= 0 && selected_camera_id < num_camera_ids);

  image_lights->lights = lights_on_image_array[selected_camera_id];
  image_lights->lights_outside_image = lights_outside_image_array[selected_camera_id];
  image_lights->num_signals = image_lights->lights->size() +
      image_lights->lights_outside_image->size();
  _cached_lights_projections_array.push_back(image_lights);
  AINFO << "selected_camera_id: " << CAMERA_ID_TO_STR.at(selected_camera_id);

  return true;
}

bool TLPreprocessor::sync_image(
    const ImageSharedPtr &image,
    const double timestamp,
    const CameraId &camera_id,
    std::shared_ptr<ImageLights> *data,
    bool *should_pub) {
  base::MutexLock lock(&_mutex);
  PERF_FUNCTION();

  double sync_time = timestamp;
  bool sync_ok = false;
  double diff_image_pose_ts = 0.0;
  double diff_image_sys_ts = 0.0;

  // sync image with cached lights projections
  if (!sync_image_with_cached_lights_projections(
      image, camera_id, sync_time, *data,
      &diff_image_pose_ts, &diff_image_sys_ts, &sync_ok)) {
    auto camera_id_str = (CAMERA_ID_TO_STR.find(camera_id) != CAMERA_ID_TO_STR.end() ?
                          CAMERA_ID_TO_STR.at(camera_id) : std::to_string(camera_id));
    AINFO << "sync_image_with_cached_lights_projections failed, "
          << "not publish image to shared data, ts: " << GLOG_TIMESTAMP(sync_time)
          << ", camera_id: " << camera_id_str;
    *should_pub = false;
    return false;
  }

  // 同步不成功，根据 camera_id 判断是否发送图像
  if (!sync_ok) {
    // 找当前工作的焦距最大的相机，图像查定位失败时默认发送该相机的图像
    CameraId max_focal_len_working_camera =
        static_cast<CameraId>(get_max_focal_len_camera_id());
    AINFO << "working camera with maximum focal length: "
          << CAMERA_ID_TO_STR.at(max_focal_len_working_camera)
          << ", _last_pub_camera_id: " << _last_pub_camera_id;
    // 在缓存的 signals_num 中根据时间戳查找当前图像时间的灯数
    size_t current_signal_num = 0;
    for (auto itr = _cached_signal_nums_array.rbegin();
        itr != _cached_signal_nums_array.rend(); ++itr) {
      if (timestamp > itr->first && timestamp - itr->first < 0.5) {
        current_signal_num = itr->second;
        break;
      }
    }
    // 如果灯数为 0 ，那么默认发焦距最大的相机的图像
    // 否则，判断上一次发的是不是也是最大焦距的图像
    if (camera_id == max_focal_len_working_camera &&
        (current_signal_num == 0 || camera_id == _last_pub_camera_id &&
            _last_pub_camera_id != CameraId::UNKNOWN)) {
      (*data).reset(new ImageLights);
      (*data)->image = image;
      // 距离查不到灯在一定时间范围以内，找不到 pose 是由于查 /tf 降频了，不做标记
      // 降低 debug 图像上 "No valid pose" 闪烁频率
      (*data)->is_pose_valid =
          (fabs(sync_time - _last_no_signals_ts) < _no_signals_interval_seconds);
      (*data)->diff_image_pose_ts = diff_image_pose_ts;
      (*data)->diff_image_sys_ts = diff_image_sys_ts;
      (*data)->timestamp = timestamp;
      (*data)->camera_id = camera_id;
      (*data)->lights.reset(new LightPtrs);
      (*data)->lights_outside_image.reset(new LightPtrs);
      // 使用查找到的灯数
      (*data)->num_signals = current_signal_num;

      AINFO << "sync image with cached lights projection failed, "
            << "no valid pose, ts: " << GLOG_TIMESTAMP(timestamp)
            << " camera_id: " << CAMERA_ID_TO_STR.at(camera_id);

    } else {  // 其他 camera_id，直接返回，不发送图像
      AINFO << "sync image with cached lights projection failed, "
            << "no valid pose, ts: " << GLOG_TIMESTAMP(timestamp)
            << " camera_id: " << CAMERA_ID_TO_STR.at(camera_id);
      *should_pub = false;
      return false;
    }
  }
  if (sync_ok) {
    AINFO << "TLPreprocessor sync ok ts: " << GLOG_TIMESTAMP(sync_time)
          << " camera_id: " << CAMERA_ID_TO_STR.at(camera_id);
    _last_output_ts = sync_time;   // update last output timestamp only when sync. is really ok
  }
  _last_pub_camera_id = camera_id;
  *should_pub = true;

  return true;
}

void TLPreprocessor::set_last_signals(const std::vector<adu::common::hdmap::Signal> &signals) {
  _last_signals = signals;
}

void TLPreprocessor::get_last_signals(std::vector<adu::common::hdmap::Signal> *signals) const {
  *signals = _last_signals;
}

void TLPreprocessor::set_last_signals_ts(double last_signals_ts) {
  _last_signals_ts = last_signals_ts;
}

void TLPreprocessor::get_last_signals_ts(double *last_signals_ts) const {
  *last_signals_ts = _last_signals_ts;
}

void TLPreprocessor::set_valid_hdmap_interval(double seconds) {
  _valid_hdmap_interval = seconds;
}

void TLPreprocessor::get_valid_hdmap_interval(double *seconds) const {
  *seconds = _valid_hdmap_interval;
}

void TLPreprocessor::set_last_pub_camera_id(CameraId camera_id) {
  _last_pub_camera_id = camera_id;
}

void TLPreprocessor::get_last_pub_camera_id(CameraId *camera_id) const {
  *camera_id = _last_pub_camera_id;
}

void TLPreprocessor::set_last_no_signals_ts(double last_no_signals_ts) {
  _last_no_signals_ts = last_no_signals_ts;
}

void TLPreprocessor::get_last_no_signals_ts(double *last_no_signals_ts) const {
  CHECK_NOTNULL(last_no_signals_ts);
  *last_no_signals_ts = _last_no_signals_ts;
}

void TLPreprocessor::set_last_output_ts(double last_output_ts) {
  _last_output_ts = last_output_ts;
}

void TLPreprocessor::get_last_output_ts(double *last_output_ts) const {
  CHECK_NOTNULL(last_output_ts);
  *last_output_ts = _last_output_ts;
}

void TLPreprocessor::set_no_signals_interval_seconds(double seconds) {
  _no_signals_interval_seconds = seconds;
}

void TLPreprocessor::get_no_signals_interval_seconds(double *seconds) const {
  *seconds = _no_signals_interval_seconds;
}

bool TLPreprocessor::set_camera_is_working_flag(const CameraId &camera_id, bool is_working) {
  const int num_camera_ids = static_cast<int>(CameraId::CAMERA_ID_COUNT) - 1;
  const int cam_id = static_cast<int>(camera_id);
  if (cam_id < 0 || cam_id >= num_camera_ids) {
    AERROR << "set_camera_is_working_flag failed, "
           << "get unknown CameraId: " << camera_id;
    return false;
  }
  _camera_is_working_flags[cam_id] = static_cast<int>(is_working);
  AINFO << "set_camera_is_working_flag succeeded, camera_id: " << CAMERA_ID_TO_STR.at(camera_id)
        << ", flag: " << _camera_is_working_flags[cam_id];

  return true;
}

bool TLPreprocessor::get_camera_is_working_flag(const CameraId &camera_id, bool *is_working) const {
  const int num_camera_ids = static_cast<int>(CameraId::CAMERA_ID_COUNT) - 1;
  const int cam_id = static_cast<int>(camera_id);
  if (cam_id < 0 || cam_id >= num_camera_ids) {
    AERROR << "get_camera_is_working_flag failed, "
           << "get unknown CameraId: " << camera_id;
    return false;
  }
  if (_camera_is_working_flags.find(cam_id) == _camera_is_working_flags.end() ||
      _camera_is_working_flags.at(cam_id) == 0) {
    *is_working = false;
    return true;
  }

  *is_working = true;
  return true;
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
                                  const std::map<int, int> &image_borders_size,
                                  CameraId *selection) {
  *selection = static_cast<CameraId>(get_max_focal_len_camera_id());
  // 找当前工作的焦距最小的相机，不进行边界检查
  int min_focal_len_working_camera = get_min_focal_len_camera_id();
  AINFO << "working camera with minimum focal length: "
        << CAMERA_ID_TO_STR.at(min_focal_len_working_camera);

  for (size_t cam_id = 0; cam_id < lights_on_image_array.size(); ++cam_id) {
    if (!lights_outside_image_array[cam_id]->empty()) {
      continue;
    }
    if (_camera_is_working_flags.find(cam_id) == _camera_is_working_flags.end() ||
        _camera_is_working_flags[cam_id] == 0) {
      continue;
    }
    bool ok = true;
    if (cam_id != min_focal_len_working_camera) {
      for (const LightPtr &light : *(lights_on_image_array[cam_id])) {
        if (is_in_bord(cv::Size(_projection_image_cols,
                                _projection_image_rows),
                       light->region.projection_roi,
                       image_borders_size.at(cam_id))) {
          ok = false;
          AINFO << "light project on image border region, "
                << "CameraId: " << CAMERA_ID_TO_STR.at(cam_id);
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

bool TLPreprocessor::project_lights(const MultiCamerasProjection &projection,
                                    const std::vector<adu::common::hdmap::Signal> &signals,
                                    const CarPose &pose,
                                    CameraId camera_id,
                                    std::shared_ptr<LightPtrs> &lights_on_image,
                                    std::shared_ptr<LightPtrs> &lights_outside_image) {
  if (signals.empty()) {
    ADEBUG << "project_lights get empty signals.";
    return true;
  }

  const int num_camera_ids = static_cast<int>(CameraId::CAMERA_ID_COUNT) - 1;
  const int cam_id = static_cast<int>(camera_id);
  if (cam_id < 0 || cam_id >= num_camera_ids) {
    AERROR << "project_lights get invalid CameraId: " << camera_id;
    return false;
  }

  // 相机未工作（或一定时间内都没收到图像），不进行投影
  bool camera_is_working = false;
  get_camera_is_working_flag(camera_id, &camera_is_working);
  if (!camera_is_working) {
    XLOG(WARN) << "TLPreprocessor::project_lights not project lights, "
               << "camera is not working, CameraId: " << CAMERA_ID_TO_STR.at(camera_id);
    return true;
  }

  for (size_t i = 0; i < signals.size(); ++i) {
    LightPtr light(new Light);
    light->info = signals[i];
    if (!projection.project(pose, ProjectOption(camera_id), light.get())) {
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
    std::shared_ptr<ImageLights> &data,
    double *diff_image_pose_ts,
    double *diff_image_sys_ts,
    bool *sync_ok) {
  PERF_FUNCTION();
  const int num_camera_ids = static_cast<int>(CameraId::CAMERA_ID_COUNT) - 1;
  const int cam_id = static_cast<int>(camera_id);
  if (cam_id < 0 || cam_id >= num_camera_ids) {
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
          (CAMERA_ID_TO_STR.find(proj_cam_id) != CAMERA_ID_TO_STR.end() ?
           CAMERA_ID_TO_STR.at(proj_cam_id) : std::to_string(proj_cam_id));
      // 找到对应时间的定位，但是相机 ID 不符
      if (camera_id != (*ptr_lights_projection)->camera_id) {
        XLOG(WARN) << "find appropriate localization, but camera_id not match"
                   << ", cached projection's camera_id: "
                   << proj_cam_id_str
                   << " , image's camera_id: "
                   << CAMERA_ID_TO_STR.at(image_cam_id);
        //return false;
        continue;
      }
      if (sync_time < _last_output_ts) {
        XLOG(WARN) << "TLPreprocessor reject the image pub ts:"
                   << GLOG_TIMESTAMP(sync_time)
                   << " which is earlier than last output ts:"
                   << GLOG_TIMESTAMP(_last_output_ts)
                   << ", image camera_id: " << CAMERA_ID_TO_STR.at(image_cam_id);
        return false;
      }
      *sync_ok = true;
      break;
    }
  }

  if (*sync_ok) {
    // elements from cached_array should not possess any pointer to image resource
    // so here we allocate new space and copy other contents
    if (data.get() == NULL) {
      data.reset(new ImageLights);
    }
    data->pose = (*ptr_lights_projection)->pose;
    data->camera_id = (*ptr_lights_projection)->camera_id;
    data->timestamp = sync_time;  // 图像时间戳
    data->is_pose_valid = (*ptr_lights_projection)->is_pose_valid;
    data->has_signals = (*ptr_lights_projection)->has_signals;
    data->diff_image_pose_ts = (*ptr_lights_projection)->diff_image_pose_ts;
    data->lights = (*ptr_lights_projection)->lights;
    data->lights_outside_image = (*ptr_lights_projection)->lights_outside_image;
    data->num_signals = (*ptr_lights_projection)->num_signals;
    data->image = image;  // (*ptr_lights_projection)->image.get() == NULL
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
            << " camera_id: " << CAMERA_ID_TO_STR.at(camera_id);
      return true;
    }
    if (sync_time < cached_array.front()->timestamp) {
      double pose_ts = cached_array.front()->timestamp;
      double system_ts = base::TimeUtil::get_current_time();
      XLOG(WARN) << "TLPreprocessor " << cached_array_str << " sync failed, image ts: "
                 << GLOG_TIMESTAMP(sync_time)
                 << ", which is earlier than " << cached_array_str << ".front() ts: "
                 << GLOG_TIMESTAMP(pose_ts)
                 << ", diff between image and pose ts: "
                 << GLOG_TIMESTAMP(sync_time - pose_ts)
                 << "; system ts: " << GLOG_TIMESTAMP(system_ts)
                 << ", diff between image and system ts: "
                 << GLOG_TIMESTAMP(sync_time - system_ts)
                 << ", camera_id: " << CAMERA_ID_TO_STR.at(camera_id);
      // difference between image and pose timestamps
      *diff_image_pose_ts = sync_time - pose_ts;
      *diff_image_sys_ts = sync_time - system_ts;
    } else if (sync_time > cached_array.back()->timestamp) {
      double pose_ts = cached_array.back()->timestamp;
      double system_ts = base::TimeUtil::get_current_time();
      XLOG(WARN) << "TLPreprocessor " << cached_array_str << " sync failed, image ts: "
                 << GLOG_TIMESTAMP(sync_time)
                 << ", which is older than " << cached_array_str << ".back() ts: "
                 << GLOG_TIMESTAMP(pose_ts)
                 << ", diff between image and pose ts: "
                 << GLOG_TIMESTAMP(sync_time - pose_ts)
                 << "; system ts: " << GLOG_TIMESTAMP(system_ts)
                 << ", diff between image and system ts: "
                 << GLOG_TIMESTAMP(sync_time - system_ts)
                 << ", camera_id: " << CAMERA_ID_TO_STR.at(camera_id);
      *diff_image_pose_ts = sync_time - pose_ts;
      *diff_image_sys_ts = sync_time - system_ts;
    } else if (!find_loc) {
      // 确实没找到定位才打 log
      XLOG(WARN) << "TLPreprocessor " << cached_array_str << " sync failed, image ts: "
                 << GLOG_TIMESTAMP(sync_time)
                 << ", cannot find close enough timestamp, "
                 << cached_array_str << ".front() ts: "
                 << GLOG_TIMESTAMP(cached_array.front()->timestamp) << ", "
                 << cached_array_str << ".back() ts: "
                 << GLOG_TIMESTAMP(cached_array.back()->timestamp)
                 << ", camera_id: " << CAMERA_ID_TO_STR.at(camera_id);
    }
  }

  return true;
}

bool TLPreprocessor::is_in_bord(const cv::Size size,
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
  int min_focal_len_working_camera = static_cast<int>(CameraId::WIDE_FOCUS);
  const int num_camera_ids = static_cast<int>(CameraId::CAMERA_ID_COUNT) - 1;
  for (int cam_id = num_camera_ids - 1; cam_id >= 0; --cam_id) {
    if (_camera_is_working_flags.find(cam_id) != _camera_is_working_flags.end() &&
        _camera_is_working_flags[cam_id] == 1) {
      min_focal_len_working_camera = cam_id;
      break;
    }
  }
  return min_focal_len_working_camera;
}

int TLPreprocessor::get_max_focal_len_camera_id() {
  int max_focal_len_working_camera = static_cast<int>(CameraId::LONG_FOCUS);
  const int num_camera_ids = static_cast<int>(CameraId::CAMERA_ID_COUNT) - 1;
  for (int cam_id = 0; cam_id < num_camera_ids; ++cam_id) {
    if (_camera_is_working_flags.find(cam_id) != _camera_is_working_flags.end() &&
        _camera_is_working_flags[cam_id] == 1) {
      max_focal_len_working_camera = cam_id;
      break;
    }
  }
  return max_focal_len_working_camera;
}

bool TLPreprocessor::select_camera_by_lights_projection(
    const double timestamp,
    const CarPose &pose,
    const std::vector<adu::common::hdmap::Signal> &signals,
    const MultiCamerasProjection &projection,
    const std::map<int, int> &image_borders_size,
    std::shared_ptr<ImageLights> *image_lights,
    bool *projections_outside_all_images,
    CameraId *selected_camera_id) {
  AINFO << "select_camera_by_lights_projection signals number: " << signals.size();

  bool has_signals = (signals.size() > 0);

  // lights projections info.
  const int num_camera_ids = static_cast<int>(CameraId::CAMERA_ID_COUNT) - 1;
  std::vector<std::shared_ptr<LightPtrs> > lights_on_image_array(
      num_camera_ids);
  std::vector<std::shared_ptr<LightPtrs> > lights_outside_image_array(
      num_camera_ids);
  for (auto &light_ptrs : lights_on_image_array) {
    light_ptrs.reset(new LightPtrs);
  }
  for (auto &light_ptrs : lights_outside_image_array) {
    light_ptrs.reset(new LightPtrs);
  }

  // project light region on each camera's image plane
  for (int cam_id = 0; cam_id < num_camera_ids; ++cam_id) {
    if (!project_lights(projection, signals, pose,
                        static_cast<CameraId>(cam_id),
                        lights_on_image_array[cam_id],
                        lights_outside_image_array[cam_id])) {
      AERROR << "select_camera_by_lights_projection project lights on "
             << CAMERA_ID_TO_STR.at(cam_id) << " image failed, "
             << "ts: " << GLOG_TIMESTAMP(timestamp)
             << ", camera_id: " << CAMERA_ID_TO_STR.at(cam_id);
      return false;
    }
  }

  // 有灯 && (lights_on_image_array 里一个灯都没有), 说明所有相机的图像里都没有灯投影
  *projections_outside_all_images = has_signals;
  for (int cam_id = 0; cam_id < num_camera_ids; ++cam_id) {
    *projections_outside_all_images = *projections_outside_all_images &&
        (lights_on_image_array[cam_id]->size() == 0);
  }

  // select which image to be used
  select_image(pose,
               lights_on_image_array,
               lights_outside_image_array,
               image_borders_size,
               selected_camera_id);
  assert(*selected_camera_id >= 0 && *selected_camera_id < num_camera_ids);

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
} // namespace adu
