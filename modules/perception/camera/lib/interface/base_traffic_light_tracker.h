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

struct TrafficLightTrackerInitOptions : public BaseInitOptions {};

struct TrafficLightTrackerOptions {
  double time_stamp;
};

class BaseTrafficLightTracker {
 public:
  BaseTrafficLightTracker() = default;

  virtual ~BaseTrafficLightTracker() = default;

  virtual bool Init(const TrafficLightTrackerInitOptions& options =
                        TrafficLightTrackerInitOptions()) = 0;

  // @brief: track detected traffic_light.
  // @param [in]: options
  // @param [in/out]: frame
  // traffic_light type and 2D bbox should be filled, required,
  virtual bool Track(const TrafficLightTrackerOptions& options,
                     CameraFrame* frame) = 0;

  virtual std::string Name() const = 0;

  BaseTrafficLightTracker(const BaseTrafficLightTracker&) = delete;
  BaseTrafficLightTracker& operator=(const BaseTrafficLightTracker&) = delete;
};  // class BaseTrafficLightTracker

PERCEPTION_REGISTER_REGISTERER(BaseTrafficLightTracker);
#define REGISTER_TRAFFIC_LIGHT_TRACKER(name) \
  PERCEPTION_REGISTER_CLASS(BaseTrafficLightTracker, name)

}  // namespace camera
}  // namespace perception
}  // namespace apollo
