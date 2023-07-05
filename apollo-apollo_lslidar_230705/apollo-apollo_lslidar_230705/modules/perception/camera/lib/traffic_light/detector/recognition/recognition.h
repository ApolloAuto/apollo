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
#include <vector>

#include "cyber/common/macros.h"
#include "modules/perception/camera/lib/interface/base_traffic_light_detector.h"
#include "modules/perception/camera/lib/traffic_light/detector/recognition/classify.h"
#include "modules/perception/inference/inference.h"
#include "modules/perception/pipeline/proto/stage/recognition.pb.h"
#include "modules/perception/pipeline/stage.h"

namespace apollo {
namespace perception {
namespace camera {

class TrafficLightRecognition final : public BaseTrafficLightDetector {
 public:
  TrafficLightRecognition() = default;

  ~TrafficLightRecognition() = default;

  bool Init(const TrafficLightDetectorInitOptions& options) override;

  // @brief: detect traffic_light from image.
  // @param [in]: options
  // @param [in/out]: frame
  // traffic_light type and 2D bbox should be filled, required,
  bool Detect(const TrafficLightDetectorOptions& options,
              CameraFrame* frame) override;

  bool Init(const StageConfig& stage_config) override;

  bool Process(DataFrame* data_frame) override;

  bool IsEnabled() const override { return enable_; }

  std::string Name() const override { return name_; }

 private:
  std::shared_ptr<ClassifyBySimple> classify_vertical_;
  std::shared_ptr<ClassifyBySimple> classify_quadrate_;
  std::shared_ptr<ClassifyBySimple> classify_horizontal_;

  TrafficLightRecognitionConfig recognize_param_;
  std::string recognition_root_dir;

  DISALLOW_COPY_AND_ASSIGN(TrafficLightRecognition);
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
