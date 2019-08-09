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

#include <fstream>
#include <map>
#include <memory>
#include <string>

#include "modules/perception/camera/app/perception.pb.h"
#include "modules/perception/camera/common/camera_frame.h"
#include "modules/perception/camera/common/object_template_manager.h"
#include "modules/perception/camera/lib/interface/base_calibration_service.h"
#include "modules/perception/camera/lib/interface/base_calibrator.h"
#include "modules/perception/camera/lib/interface/base_camera_perception.h"
#include "modules/perception/camera/lib/interface/base_feature_extractor.h"
#include "modules/perception/camera/lib/interface/base_init_options.h"
#include "modules/perception/camera/lib/interface/base_lane_detector.h"
#include "modules/perception/camera/lib/interface/base_lane_postprocessor.h"
#include "modules/perception/camera/lib/interface/base_obstacle_detector.h"
#include "modules/perception/camera/lib/interface/base_obstacle_postprocessor.h"
#include "modules/perception/camera/lib/interface/base_obstacle_tracker.h"
#include "modules/perception/camera/lib/interface/base_obstacle_transformer.h"

namespace apollo {
namespace perception {
namespace camera {

class ObstacleCameraPerception : public BaseCameraPerception {
 public:
  ObstacleCameraPerception()
      : transformer_(nullptr),
        tracker_(nullptr),
        extractor_(nullptr),
        lane_detector_(nullptr),
        lane_postprocessor_(nullptr),
        calibration_service_(nullptr),
        object_template_manager_(nullptr) {}
  ObstacleCameraPerception(const ObstacleCameraPerception &) = delete;
  ObstacleCameraPerception &operator=(const ObstacleCameraPerception &) =
      delete;
  ~ObstacleCameraPerception() = default;
  bool Init(const CameraPerceptionInitOptions &options) override;
  void InitLane(const std::string &work_root,
                const app::PerceptionParam &perception_param);
  void InitCalibrationService(const std::string &work_root,
                              const base::BaseCameraModelPtr model,
                              const app::PerceptionParam &perception_param);
  void SetCameraHeightAndPitch(
      const std::map<std::string, float> &name_camera_ground_height_map,
      const std::map<std::string, float> &name_camera_pitch_angle_diff_map,
      const float &pitch_angle_calibrator_working_sensor);
  void SetIm2CarHomography(Eigen::Matrix3d homography_im2car);
  bool GetCalibrationService(BaseCalibrationService **calibration_service);
  bool Perception(const CameraPerceptionOptions &options,
                  CameraFrame *frame) override;
  std::string Name() const override { return "ObstacleCameraPerception"; }

 private:
  std::map<std::string, Eigen::Matrix3f> name_intrinsic_map_;
  std::map<std::string, std::shared_ptr<BaseObstacleDetector>>
      name_detector_map_;
  std::shared_ptr<BaseObstacleTransformer> transformer_;
  std::shared_ptr<BaseObstaclePostprocessor> obstacle_postprocessor_;
  std::shared_ptr<BaseObstacleTracker> tracker_;
  std::shared_ptr<BaseFeatureExtractor> extractor_;
  std::shared_ptr<BaseLaneDetector> lane_detector_;
  std::shared_ptr<BaseLanePostprocessor> lane_postprocessor_;
  std::shared_ptr<BaseCalibrationService> calibration_service_;
  app::PerceptionParam perception_param_;
  std::ofstream out_track_;
  std::ofstream out_pose_;
  std::string lane_calibration_working_sensor_name_ = "";
  bool write_out_lane_file_ = false;
  bool write_out_calib_file_ = false;
  std::string out_lane_dir_;
  std::string out_calib_dir_;

 protected:
  ObjectTemplateManager *object_template_manager_ = nullptr;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
