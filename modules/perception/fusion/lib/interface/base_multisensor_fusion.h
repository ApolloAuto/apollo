/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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
#include <vector>

#include "cyber/common/macros.h"
#include "modules/perception/fusion/base/base_forward_declaration.h"
#include "modules/perception/fusion/base/scene.h"
#include "modules/perception/fusion/base/sensor_frame.h"
#include "modules/perception/lib/registerer/registerer.h"

namespace apollo {
namespace perception {
namespace fusion {

struct ObstacleMultiSensorFusionParam {
  std::string main_sensor;
  std::string fusion_method;
};

class BaseMultiSensorFusion {
 public:
  BaseMultiSensorFusion() = default;
  virtual ~BaseMultiSensorFusion() = default;

  virtual bool Init(const ObstacleMultiSensorFusionParam& param) = 0;

  virtual bool Process(const base::FrameConstPtr& frame,
               std::vector<base::ObjectPtr>* objects) = 0;

  virtual std::string Name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseMultiSensorFusion);
};  // Class BaseMultiSensorFusion

PERCEPTION_REGISTER_REGISTERER(BaseMultiSensorFusion);
#define PERCEPTION_REGISTER_MULTISENSORFUSION(name) \
  PERCEPTION_REGISTER_CLASS(BaseMultiSensorFusion, name)

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
