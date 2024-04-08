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

#include <memory>
#include <vector>

#include "modules/perception/common/base/traffic_light.h"
#include "modules/perception/common/camera/common/data_provider.h"

namespace apollo {
namespace perception {
namespace camera {

struct TrafficLightFrame {
  // timestamp
  double timestamp = 0.0;
  // frame sequence id
  int frame_id = 0;
  // data provider
  std::shared_ptr<camera::DataProvider> data_provider;
  // detected traffic lights
  std::vector<base::TrafficLightPtr> traffic_lights;
};  // struct TrafficLightFrame

}  // namespace camera
}  // namespace perception
}  // namespace apollo
