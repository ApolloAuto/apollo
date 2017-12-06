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
#ifndef MODULES_PERCEPTION_TRAFFIC_LIGHT_PROJECTION_MULTI_CAMERA_PROJECTION_H_
#define MODULES_PERCEPTION_TRAFFIC_LIGHT_PROJECTION_MULTI_CAMERA_PROJECTION_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/traffic_light/interface/base_projection.h"

namespace apollo {
namespace perception {
namespace traffic_light {
// @brief 2 Camera Projection project the Light into the image.
class MultiCamerasProjection {
 public:
  MultiCamerasProjection() = default;

  virtual ~MultiCamerasProjection() = default;
  virtual bool Init();
  virtual bool Project(const CarPose &pose, const ProjectOption &option,
                       Light *light) const;
  std::string name() const { return "TLPreprocessor"; }

 private:
  std::vector<CameraCoeffient> camera_coeffient_;
  std::vector<std::string> camera_names_;
  std::unique_ptr<BaseProjection> projection_;
};

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo

#endif
