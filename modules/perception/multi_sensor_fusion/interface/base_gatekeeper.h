/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#pragma once

#include <string>

#include "cyber/common/macros.h"
#include "modules/perception/common/lib/interface/base_init_options.h"
#include "modules/perception/common/lib/registerer/registerer.h"
#include "modules/perception/multi_sensor_fusion/base/base_forward_declaration.h"
#include "modules/perception/multi_sensor_fusion/base/scene.h"
#include "modules/perception/multi_sensor_fusion/base/sensor_frame.h"

namespace apollo {
namespace perception {
namespace fusion {

struct GatekeeperInitOptions : public BaseInitOptions {};

class BaseGatekeeper {
 public:
  BaseGatekeeper() = default;
  virtual ~BaseGatekeeper() = default;

  /**
   * @brief Init base gate keeper config
   *
   * @param options
   * @return true
   * @return false
   */
  virtual bool Init(const GatekeeperInitOptions& options) = 0;

  /**
   * @brief whether to allow publishing based on conditions
   *
   * @param track
   * @return true
   * @return false
   */
  virtual bool AbleToPublish(const TrackPtr& track) = 0;

  /**
   * @brief The name of BaseGatekeeper
   *
   * @return std::string
   */
  virtual std::string Name() const = 0;

  DISALLOW_COPY_AND_ASSIGN(BaseGatekeeper);
};

PERCEPTION_REGISTER_REGISTERER(BaseGatekeeper);
#define PERCEPTION_REGISTER_GATEKEEPER(name) \
  PERCEPTION_REGISTER_CLASS(BaseGatekeeper, name)

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
