/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/camera_location_estimation/proto/camera_location_estimation.pb.h"

#include "cyber/cyber.h"
#include "modules/perception/camera_location_estimation/interface/base_transformer.h"
#include "modules/perception/common/onboard/inner_component_messages/camera_detection_component_messages.h"

namespace apollo {
namespace perception {
namespace camera {

class CameraLocationEstimationComponent final
    : public cyber::Component<onboard::CameraFrame> {
 public:
  /**
   * @brief Init for camera location estimatation component.
   *
   * @return true
   * @return false
   */
  bool Init() override;
  /**
   * @brief Process of camera location estimatation component.
   *
   * @param msg image msg
   * @return true
   * @return false
   */
  bool Proc(const std::shared_ptr<onboard::CameraFrame>& msg) override;

 private:
  void InitTransformer(
      const CameraLocationEstimation& location_estimation_param);

 private:
  std::shared_ptr<BaseTransformer> transformer_;

  std::shared_ptr<cyber::Writer<onboard::CameraFrame>> writer_;
};

CYBER_REGISTER_COMPONENT(CameraLocationEstimationComponent);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
