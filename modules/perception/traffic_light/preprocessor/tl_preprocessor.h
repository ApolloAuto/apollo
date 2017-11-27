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

#ifndef MODULES_PERCEPTION_TRAFFIC_LIGHT_TL_PREPROCESSOR_H
#define MODULES_PERCEPTION_TRAFFIC_LIGHT_TL_PREPROCESSOR_H

#include <memory>
#include <vector>
#include <map>
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lib/base/mutex.h"

#include "modules/perception/traffic_light/interface/base_preprocessor.h"

#include "modules/perception/lib/base/timer.h"
#include "modules/perception/traffic_light/base/image.h"
#include "modules/perception/traffic_light/base/image_lights.h"
#include "modules/perception/traffic_light/projection/multi_camera_projection.h"

namespace apollo {
namespace perception {
namespace traffic_light {

class TLPreprocessor : public BasePreprocessor {
 public:
  TLPreprocessor() : kCountCameraId(static_cast<int>(CAMERA_ID_COUNT)),
                     kLongFocusIdx(static_cast<int>(LONG_FOCUS)),
                     kShortFocusIdx(static_cast<int>(SHORT_FOCUS)) {}
  ~TLPreprocessor() = default;

  virtual bool Init();

  virtual std::string Name() const {
    return "TLPreprocessor";
  }

  bool AddCachedLightsProjections(const CarPose &pose,
                                  const std::vector<apollo::hdmap::Signal> &signals,
                                  const std::map<int, int> &image_borders_size,
                                  const double ts,
                                  bool *projections_outside_all_images);

  bool SyncImage(
      const ImageSharedPtr &image,
      const double timestamp,
      const CameraId &camera_id,
      std::shared_ptr<ImageLights> *pub_data,
      bool *should_pub);

  void SetLastSignals(const std::vector<apollo::hdmap::Signal> &signals);
  void GetLastSignals(std::vector<apollo::hdmap::Signal> *signals) const;

  void set_last_signals_ts(double last_signals_ts);
  void get_last_signals_ts(double *last_signals_ts) const;

  void set_valid_hdmap_interval(double seconds);
  void get_valid_hdmap_interval(double *seconds) const;

  void set_last_pub_camera_id(CameraId camera_id);
  void get_last_pub_camera_id(CameraId *camera_id) const;

  void set_no_signals_interval_seconds(double seconds);
  void get_no_signals_interval_seconds(double *seconds) const;

  bool set_camera_is_working_flag(const CameraId &camera_id, bool is_working);
  bool get_camera_is_working_flag(const CameraId &camera_id, bool *is_working) const;

  bool set_max_cached_image_lights_array_size(size_t max_cached_image_lights_array_size);
  bool get_max_cached_image_lights_array_size(size_t *max_cached_image_lights_array_size) const;

  bool select_camera_by_lights_projection(const double timestamp,
                                          const CarPose &pose,
                                          const std::vector<apollo::hdmap::Signal> &signals,
                                          const std::map<int, int> &image_borders_size,
                                          std::shared_ptr<ImageLights> *image_lights,
                                          bool *projections_outside_all_images,
                                          CameraId *selected_camera_id);

 private:
  void select_image(const CarPose &pose,
                    const std::vector<std::shared_ptr<LightPtrs> > &lights_on_image_array,
                    const std::vector<std::shared_ptr<LightPtrs> > &lights_outside_image_array,
                    const std::map<int, int> &image_borders_size,
                    CameraId *selection);

  //@brief Project lights from HDMap onto long focus or short focus image plane
  bool project_lights(const std::vector<apollo::hdmap::Signal> &signals,
                      const CarPose &pose,
                      CameraId camera_id,
                      std::shared_ptr<LightPtrs> &lights_on_image,
                      std::shared_ptr<LightPtrs> &lights_outside_image);

  //@brief Sync. image with cached lights projections
  bool sync_image_with_cached_lights_projections(
      const ImageSharedPtr &image,
      CameraId camera_id,
      double timestamp,
      std::shared_ptr<ImageLights> &data,
      double *diff_image_pose_ts,
      double *diff_image_sys_ts,
      bool *sync_ok);

  bool is_on_border(const cv::Size size, const cv::Rect &roi, const int border_size) const;

  int get_min_focal_len_camera_id();
  int get_max_focal_len_camera_id();

 private:
  MultiCamerasProjection _projection;
  std::vector<apollo::hdmap::Signal> _last_signals;
  double _last_signals_ts = -1;
  double _valid_hdmap_interval = 1.5;

  double _last_no_signals_ts = -1;

  CameraId _last_pub_camera_id = CameraId::UNKNOWN;

  double _last_output_ts = 0.0;

  std::vector<std::shared_ptr<ImageLights> > _cached_lights_projections_array;

  std::vector<int> _camera_is_working_flags;
  Mutex _mutex;

  // cache signal numbers
  size_t _max_cached_signal_nums_size = 200;
  std::vector<std::pair<double, size_t> > _cached_signal_nums_array;

  // some parameters from config file
  int _max_cached_image_lights_array_size = 100;
  int _projection_image_cols = 1920;
  int _projection_image_rows = 1080;
  double _sync_interval_seconds = 0.1;
  double _no_signals_interval_seconds = 0.5;
  const int kCountCameraId;
  const int kLongFocusIdx;
  const int kShortFocusIdx;

 DISALLOW_COPY_AND_ASSIGN(TLPreprocessor);
};

} // namespace traffic_light
} // namespace perception
} // namespace apollo

#endif  // MODULES_PERCEPTION_TRAFFIC_LIGHT_TL_PREPROCESSOR_H
