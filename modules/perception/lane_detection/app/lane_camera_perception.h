/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <fstream>
#include <map>
#include <memory>
#include <string>

#include "modules/perception/lane_detection/proto/perception.pb.h"

#include "cyber/common/macros.h"
#include "modules/common/util/eigen_defs.h"
#include "modules/perception/common/camera/common/camera_frame.h"
#include "modules/perception/common/camera/common/object_template_manager.h"
#include "modules/perception/common/lib/interface/base_calibration_service.h"
#include "modules/perception/common/lib/interface/base_calibrator.h"
#include "modules/perception/common/lib/interface/base_init_options.h"
#include "modules/perception/lane_detection/interface/base_camera_perception.h"
#include "modules/perception/lane_detection/interface/base_lane_detector.h"
#include "modules/perception/lane_detection/interface/base_lane_postprocessor.h"

namespace apollo {
namespace perception {
namespace camera {

class LaneCameraPerception final : public BaseCameraPerception {
 public:
  template <typename T, class EigenType>
  using EigenMap = apollo::common::EigenMap<T, EigenType>;

 public:
  LaneCameraPerception() = default;
  ~LaneCameraPerception() = default;

  bool Init(const CameraPerceptionInitOptions &options) override;
  void InitLane(base::BaseCameraModelPtr &model,
                const app::PerceptionParam &perception_param);
  void InitCalibrationService(const base::BaseCameraModelPtr model,
                              const app::PerceptionParam &perception_param);
  void SetCameraHeightAndPitch(
      const std::map<std::string, float> name_camera_ground_height_map,
      const std::map<std::string, float> name_camera_pitch_angle_diff_map,
      const float &pitch_angle_calibrator_working_sensor);
  void SetIm2CarHomography(const Eigen::Matrix3d &homography_im2car);
  bool GetCalibrationService(BaseCalibrationService **calibration_service);
  bool Perception(const CameraPerceptionOptions &options,
                  CameraFrame *frame) override;

  std::string Name() const override { return "LaneCameraPerception"; }

 private:
  EigenMap<std::string, Eigen::Matrix3f> name_intrinsic_map_;
  EigenMap<std::string, Eigen::Matrix4d> name_extrinsic_map_;
  std::shared_ptr<BaseLaneDetector> lane_detector_;
  std::shared_ptr<BaseLanePostprocessor> lane_postprocessor_;
  std::shared_ptr<BaseCalibrationService> calibration_service_;
  app::PerceptionParam perception_param_;
  std::string lane_calibration_working_sensor_name_ = "";
  bool write_out_lane_file_ = false;
  bool write_out_calib_file_ = false;
  std::string out_lane_dir_;
  std::string out_calib_dir_;

  DISALLOW_COPY_AND_ASSIGN(LaneCameraPerception);
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
