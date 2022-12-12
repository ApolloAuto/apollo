/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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
#include <vector>

#include "modules/perception/camera/app/proto/perception.pb.h"
#include "modules/perception/camera/lib/obstacle/detector/smoke/proto/smoke.pb.h"

#include "cyber/common/macros.h"
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
#include "modules/perception/pipeline/pipeline.h"

namespace apollo {
namespace perception {
namespace camera {

class ObstacleDetectionCamera final : public BaseCameraPerception {
 public:
  using CameraDetectionConfig = pipeline::CameraDetectionConfig;
  using StageType = pipeline::StageType;

 public:
  ObstacleDetectionCamera() = default;
  ~ObstacleDetectionCamera() = default;

  bool Init(const CameraPerceptionInitOptions &options) override;

  bool Init(const PipelineConfig &pipeline_config) override;

  bool Perception(const CameraPerceptionOptions &options,
                  CameraFrame *frame) override;

  bool Process(DataFrame *data_frame) override;

  std::string Name() const override { return name_; }

 protected:
  ObjectTemplateManager *object_template_manager_;

 private:
  std::map<std::string, Eigen::Matrix3f> name_intrinsic_map_;
  std::map<std::string, std::shared_ptr<BaseObstacleDetector>>
      name_detector_map_;
  std::shared_ptr<BaseObstacleTransformer> transformer_;
  std::shared_ptr<BaseObstaclePostprocessor> obstacle_postprocessor_;
  std::shared_ptr<BaseObstacleTracker> tracker_;

  app::PerceptionParam perception_param_;
  smoke::SmokeParam smoke_param_;
  std::ofstream out_track_;
  std::ofstream out_pose_;
  std::vector<std::string> camera_names_;

  CameraDetectionConfig camera_detection_config_;

  DISALLOW_COPY_AND_ASSIGN(ObstacleDetectionCamera);
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
