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

#include <memory>
#include <string>

#include "cyber/common/macros.h"
#include "modules/perception/camera/app/proto/perception.pb.h"
#include "modules/perception/camera/common/camera_frame.h"
#include "modules/perception/camera/lib/interface/base_camera_perception.h"
#include "modules/perception/camera/lib/interface/base_feature_extractor.h"
#include "modules/perception/camera/lib/interface/base_init_options.h"
#include "modules/perception/camera/lib/interface/base_traffic_light_detector.h"
#include "modules/perception/camera/lib/interface/base_traffic_light_tracker.h"

namespace apollo {
namespace perception {
namespace camera {

class TrafficLightCameraPerception final : public BaseCameraPerception {
 public:
  TrafficLightCameraPerception() = default;
  ~TrafficLightCameraPerception() = default;

  bool Init(const CameraPerceptionInitOptions &options) override;
  bool Perception(const CameraPerceptionOptions &options,
                  CameraFrame *frame) override;

  bool Init(const PipelineConfig& pipeline_config) override;

  bool Process(DataFrame* data_frame) override;

  std::string Name() const override { return name_; }

 private:
  std::shared_ptr<BaseTrafficLightDetector> detector_;
  std::shared_ptr<BaseTrafficLightDetector> recognizer_;
  std::shared_ptr<BaseTrafficLightTracker> tracker_;
  app::TrafficLightParam tl_param_;

  DISALLOW_COPY_AND_ASSIGN(TrafficLightCameraPerception);
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
