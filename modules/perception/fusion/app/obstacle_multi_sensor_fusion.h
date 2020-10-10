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
#include <vector>

#include "modules/perception/fusion/lib/interface/base_fusion_system.h"

namespace apollo {
namespace perception {
namespace fusion {

struct ObstacleMultiSensorFusionParam {
  std::string main_sensor;
  std::string fusion_method;
};

class ObstacleMultiSensorFusion {
 public:
  ObstacleMultiSensorFusion() = default;
  ~ObstacleMultiSensorFusion() = default;
  ObstacleMultiSensorFusion(const ObstacleMultiSensorFusion&) = delete;
  ObstacleMultiSensorFusion& operator=(const ObstacleMultiSensorFusion&) =
      delete;
  bool Init(const ObstacleMultiSensorFusionParam& param);
  bool Process(const base::FrameConstPtr& frame,
               std::vector<base::ObjectPtr>* objects);

  std::string Name() const { return "ObstacleMultiSensorFusion"; }

 protected:
  BaseFusionSystem* fusion_ = nullptr;
};

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
