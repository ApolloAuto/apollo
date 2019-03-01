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
#pragma once

#include <string>

#include "modules/perception/camera/common/camera_frame.h"
#include "modules/perception/lib/registerer/registerer.h"

#include "modules/perception/camera/lib/interface/base_init_options.h"

namespace apollo {
namespace perception {
namespace camera {

struct LaneTrackerInitOptions : public BaseInitOptions {};

struct LaneTrackerOptions {};

class BaseLaneTracker {
 public:
  BaseLaneTracker() = default;

  virtual ~BaseLaneTracker() = default;

  virtual bool Init(
      const LaneTrackerInitOptions& options = LaneTrackerInitOptions()) = 0;

  // @brief: track detected lanes.
  // @param [in]: options
  // @param [in/out]: frame
  // 3D information of detected lanes should be refined.
  virtual bool Track(const LaneTrackerOptions& options, CameraFrame* frame) = 0;

  virtual std::string Name() const = 0;

  BaseLaneTracker(const BaseLaneTracker&) = delete;
  BaseLaneTracker& operator=(const BaseLaneTracker&) = delete;
};  // class BaseLaneTracker

PERCEPTION_REGISTER_REGISTERER(BaseLaneTracker);
#define REGISTER_LANE_TRACKER(name) \
  PERCEPTION_REGISTER_CLASS(BaseLaneTracker, name)

}  // namespace camera
}  // namespace perception
}  // namespace apollo
