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

#include "cyber/common/macros.h"
#include "modules/perception/common/camera/common/trafficlight_frame.h"
#include "modules/perception/common/lib/interface/base_init_options.h"
#include "modules/perception/common/lib/registerer/registerer.h"

namespace apollo {
namespace perception {
namespace trafficlight {


struct TrafficLightTrackerInitOptions : public BaseInitOptions {};

class BaseTrafficLightTracker {
 public:
  /**
   * @brief Construct a new base traffic light tracker object.
   * 
   */
  BaseTrafficLightTracker() = default;
  /**
   * @brief Destroy the base traffic light tracker object.
   * 
   */
  virtual ~BaseTrafficLightTracker() = default;
  /**
   * @brief Initialize traffic light tracker parameters.
   * 
   * @param options 
   * @return true 
   * @return false 
   */
  virtual bool Init(const TrafficLightTrackerInitOptions& options =
                        TrafficLightTrackerInitOptions()) = 0;

  /**
   * @brief Track detected traffic_light.
   * 
   * @param frame 
   * @return true 
   * @return false 
   */
  virtual bool Track(camera::TrafficLightFrame* frame) = 0;

  DISALLOW_COPY_AND_ASSIGN(BaseTrafficLightTracker);
};  // class BaseTrafficLightTracker

PERCEPTION_REGISTER_REGISTERER(BaseTrafficLightTracker);
#define REGISTER_TRAFFIC_LIGHT_TRACKER(name) \
  PERCEPTION_REGISTER_CLASS(BaseTrafficLightTracker, name)

}  // namespace trafficlight
}  // namespace perception
}  // namespace apollo
