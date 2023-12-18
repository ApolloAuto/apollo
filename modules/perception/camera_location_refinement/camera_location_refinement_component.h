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

#include <map>
#include <memory>
#include <string>

#include "modules/perception/camera_location_refinement/proto/camera_location_refinement.pb.h"

#include "cyber/cyber.h"
#include "modules/common/util/eigen_defs.h"
#include "modules/perception/camera_location_refinement/interface/base_postprocessor.h"
#include "modules/perception/common/lib/interface/base_calibration_service.h"
#include "modules/perception/common/onboard/inner_component_messages/camera_detection_component_messages.h"

namespace apollo {
namespace perception {
namespace camera {

class CameraLocationRefinementComponent final
    : public cyber::Component<onboard::CameraFrame> {
 public:
  /**
   * @brief Init for camera location refinement component.
   *
   * @return true
   * @return false
   */
  bool Init() override;
  /**
   * @brief Process of camera location refinement component.
   *
   * @param msg image msg
   * @return true
   * @return false
   */
  bool Proc(const std::shared_ptr<onboard::CameraFrame>& msg) override;

 private:
  void InitCalibrationService(
      const CameraLocationRefinement& location_refinement_param);

  void InitPostprocessor(
      const CameraLocationRefinement& location_refinement_param);

  void SetCameraHeightAndPitch(
      const std::map<std::string, float>& name_camera_ground_height_map,
      const std::map<std::string, float>& name_camera_pitch_angle_diff_map,
      const float& pitch_angle_calibrator_working_sensor);

 private:
  std::shared_ptr<BasePostprocessor> postprocessor_;
  std::shared_ptr<BaseCalibrationService> calibration_service_;

  std::shared_ptr<cyber::Writer<onboard::CameraFrame>> writer_;

  apollo::common::EigenMap<std::string, Eigen::Matrix3f> name_intrinsic_map_;
};

CYBER_REGISTER_COMPONENT(CameraLocationRefinementComponent);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
