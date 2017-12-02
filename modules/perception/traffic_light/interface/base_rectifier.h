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

#ifndef MODULES_PERCEPTION_TRAFFIC_LIGHT_INTERFACE_BASE_RECTIFIER_H
#define MODULES_PERCEPTION_TRAFFIC_LIGHT_INTERFACE_BASE_RECTIFIER_H

#include <string>
#include <vector>
#include "modules/perception/lib/base/registerer.h"
#include "modules/perception/traffic_light/base/image.h"
#include "modules/perception/traffic_light/base/light.h"

namespace apollo {
namespace perception {
namespace traffic_light {

struct RectifyOption {
  CameraId camera_id = UNKNOWN;
};

//  @brief Rectifier receives the Region of lights from HD-Map,
//         While the region may be too large or not accuray.
//         Rectifier should rectify the region,
//         send the accuray regions to classifier.
class BaseRectifier {
 public:
  BaseRectifier() = default;

  virtual ~BaseRectifier() = default;

  virtual bool Init() = 0;

  // @brief: rectify light region from image or part of it
  // @param [in] const Image&: input image
  // @param [in] const RectifyOption&: rectify options
  // @param [in/out] Lights
  // @return  bool
  virtual bool Rectify(const Image &image, const RectifyOption &option,
                       std::vector<LightPtr> *lights) = 0;

  // @brief name
  virtual std::string name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseRectifier);
};

REGISTER_REGISTERER(BaseRectifier);
#define REGISTER_RECTIFIER(name) REGISTER_CLASS(BaseRectifier, name)

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_TRAFFIC_LIGHT_INTERFACE_BASE_RECTIFIER_H
