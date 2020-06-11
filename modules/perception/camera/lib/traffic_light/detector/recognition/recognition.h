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

#include "modules/perception/camera/lib/interface/base_traffic_light_detector.h"
#include "modules/perception/camera/lib/traffic_light/detector/recognition/classify.h"
#include "modules/perception/camera/lib/traffic_light/detector/recognition/recognition.pb.h"
#include "modules/perception/inference/inference.h"

namespace apollo {
namespace perception {
namespace camera {

class TrafficLightRecognition : public BaseTrafficLightDetector {
 public:
  TrafficLightRecognition() {}

  ~TrafficLightRecognition() {}

  bool Init(const TrafficLightDetectorInitOptions& options) override;

  // @brief: detect traffic_light from image.
  // @param [in]: options
  // @param [in/out]: frame
  // traffic_light type and 2D bbox should be filled, required,
  bool Detect(const TrafficLightDetectorOptions& options,
              CameraFrame* frame) override;

  std::string Name() const override;

  explicit TrafficLightRecognition(const BaseTrafficLightDetector&) = delete;
  TrafficLightRecognition& operator=(const BaseTrafficLightDetector&) = delete;

 private:
  std::shared_ptr<ClassifyBySimple> classify_vertical_;
  std::shared_ptr<ClassifyBySimple> classify_quadrate_;
  std::shared_ptr<ClassifyBySimple> classify_horizontal_;
  traffic_light::recognition::RecognizeBoxParam recognize_param_;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
