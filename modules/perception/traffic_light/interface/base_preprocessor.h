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

#ifndef MODULES_PERCEPTION_TRAFFIC_LIGHT_INTERFACE_BASE_PREPROCESSOR_H
#define MODULES_PERCEPTION_TRAFFIC_LIGHT_INTERFACE_BASE_PREPROCESSOR_H
#include <string>
#include <vector>

#include "modules/perception/lib/base/registerer.h"
#include "modules/perception/traffic_light/base/image.h"
#include "modules/perception/traffic_light/base/light.h"

namespace apollo {
namespace perception {
namespace traffic_light {

//@brief Reviser is the class is to revise the perception result.
//       It may use history info(Tracker) or some else info.
class BasePreprocessor {
 public:
  BasePreprocessor() = default;

  virtual ~BasePreprocessor() = default;

  //@brief init the reviser.
  virtual bool Init() = 0;

  //@brief Revise's name
  virtual std::string Name() const = 0;
};

REGISTER_REGISTERER(BasePreprocessor);
#define REGISTER_PREPROCESSOR(name) REGISTER_CLASS(BasePreprocessor, name)

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_TRAFFIC_LIGHT_INTERFACE_BASE_PREPROCESSOR_H
