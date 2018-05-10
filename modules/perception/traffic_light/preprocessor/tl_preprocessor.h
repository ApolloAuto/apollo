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

#ifndef MODULES_PERCEPTION_TRAFFIC_LIGHT_TL_PREPROCESSOR_H_
#define MODULES_PERCEPTION_TRAFFIC_LIGHT_TL_PREPROCESSOR_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "modules/perception/lib/base/mutex.h"
#include "modules/common/time/timer.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/traffic_light/base/image.h"
#include "modules/perception/traffic_light/base/image_lights.h"
#include "modules/perception/traffic_light/interface/base_preprocessor.h"
#include "modules/perception/traffic_light/projection/multi_camera_projection.h"

namespace apollo {
namespace perception {
namespace traffic_light {

using apollo::hdmap::Signal;
typedef std::vector<std::shared_ptr<LightPtrs>> LightsArray;

/**
 * @class TLPreprocessor
 * @brief select camera
 *        project lights
 *        cache and sync light and image
 */
class TLPreprocessor : public BasePreprocessor {
 public:
  TLPreprocessor() {}
  ~TLPreprocessor() = default;

  virtual bool Init();

  virtual std::string name() const { return "TLPreprocessor"; }

  /**
   * @brief project and cached lights before images arrive
   * @param pose used for projection
   * @param signals obtained from hdmap
   * @param timestamp
   * @return success?
   */
  bool CacheLightsProjections(const CarPose &pose,
                              const std::vector<Signal> &signals,
                              const double ts);

  /**
   * @brief when image arrives, sync image with cached lights
   * @param image
   * @param image_lights hold selected camera ,image and lights
   * @param should_pub tells whether publish this image to proc
   * @return success?
   */
  bool SyncImage(const ImageSharedPtr &image, ImageLightsPtr *image_lights,
                 bool *should_pub);

  void set_last_pub_camera_id(CameraId camera_id);
  CameraId last_pub_camera_id() const;

  int max_cached_lights_size() const;

  /**
   * @brief Project lights from HDMap onto long focus or short focus image plane
   * @param pose
   * @param signals
   * @param camera_id
   * @param lights_on_image
   * @param lights_outside_image
   * @return
   */
  bool ProjectLights(const CarPose &pose, const std::vector<Signal> &signals,
                     const CameraId &camera_id, LightPtrs *lights_on_image,
                     LightPtrs *lights_outside_image);

  /**
   * @brief given projected lights, select which camera to use
   * @param pose
   * @param lights_on_image_array
   * @param lights_outside_image_array
   * @param selectted camera
   */
  void SelectImage(const CarPose &pose,
                   const LightsArray &lights_on_image_array,
                   const LightsArray &lights_outside_image_array,
                   CameraId *selection);

  /**
   * @brief given image border size, judge whether roi is on border
   * @param size
   * @param roi
   * @param border_size
   * @return
   */
  bool IsOnBorder(const cv::Size size, const cv::Rect &roi,
                  const int border_size) const;

  int GetMinFocalLenCameraId();
  int GetMaxFocalLenCameraId();

 private:
  MultiCamerasProjection projection_;

  double last_no_signals_ts_ = -1.0;

  CameraId last_pub_camera_id_ = CameraId::UNKNOWN;

  double last_output_ts_ = 0.0;

  std::vector<std::shared_ptr<ImageLights>> cached_lights_;

  Mutex mutex_;

  // some parameters from config file
  int max_cached_lights_size_ = 100;
  int projection_image_cols_ = 1920;
  int projection_image_rows_ = 1080;
  float sync_interval_seconds_ = 0.1;
  float no_signals_interval_seconds_ = 0.5;

  DISALLOW_COPY_AND_ASSIGN(TLPreprocessor);
};
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_TRAFFIC_LIGHT_TL_PREPROCESSOR_H_
