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
#pragma once

#include <memory>
#include <string>

#include "cyber/common/macros.h"
#include "modules/perception/camera_tracking/common/camera_tracking_frame.h"
#include "modules/perception/common/lib/interface/base_init_options.h"
#include "modules/perception/common/lib/registerer/registerer.h"

namespace apollo {
namespace perception {
namespace camera {

struct ObstacleTrackerInitOptions : public BaseInitOptions {
  float image_width;
  float image_height;
};

struct ObstacleTrackerOptions {};

class BaseObstacleTracker {
 public:
  BaseObstacleTracker() = default;

  virtual ~BaseObstacleTracker() = default;

  virtual bool Init(const ObstacleTrackerInitOptions& options =
                        ObstacleTrackerInitOptions()) = 0;

  // @brief: image feature extract.
  // @param [in/out]: frame
  virtual bool FeatureExtract(CameraTrackingFrame* frame) = 0;

  // @brief: predict candidate obstales in the new image.
  // @param [in/out]: frame
  // candidate obstacle 2D boxes should be filled, required.
  virtual bool Predict(CameraTrackingFrame* frame) = 0;

  // @brief: calculate similarity  of two objects.
  // @param [in/out]: frame
  // virtual bool CalSimilarity(CameraTrackingFrame* frame) = 0;

  // @brief: associate obstales by 2D&3D information.
  // @param [in]: options
  // @param [in/out]: frame
  // virtual bool Associate(const ObstacleTrackerOptions &options,
  //             CameraTrackingFrame *frame) = 0;

  // @brief: associate obstales by 2D information.
  // @param [in/out]: frame
  // associated obstacles with tracking id should be filled, required,
  // smoothed 2D&3D information can be filled, optional.
  virtual bool Associate2D(std::shared_ptr<CameraTrackingFrame> frame) = 0;

  // @brief: associate obstales by 3D information.
  // @param [in/out]: frame
  // associated obstacles with tracking id should be filled, required,
  // smoothed 3D information can be filled, optional.
  virtual bool Associate3D(std::shared_ptr<CameraTrackingFrame> frame) = 0;

  // todo(wxt): use CameraTrackingFrame or custom object
  // virtual bool Process(const ObstacleTrackerInitOptions& options,
  // CameraTrackingFrame* camera_frame) = 0;
  virtual bool Process(std::shared_ptr<CameraTrackingFrame> camera_frame) = 0;

  DISALLOW_COPY_AND_ASSIGN(BaseObstacleTracker);
};  // class BaseObstacleTracker

PERCEPTION_REGISTER_REGISTERER(BaseObstacleTracker);
#define REGISTER_OBSTACLE_TRACKER(name) \
  PERCEPTION_REGISTER_CLASS(BaseObstacleTracker, name)

}  // namespace camera
}  // namespace perception
}  // namespace apollo
