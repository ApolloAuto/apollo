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

#ifndef MODULES_PERCEPTION_TRAFFIC_LIGHT_INTERFACE_BASE_RECOGNIZER_H
#define MODULES_PERCEPTION_TRAFFIC_LIGHT_INTERFACE_BASE_RECOGNIZER_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include "modules/perception/lib/base/registerer.h"
#include "modules/perception/traffic_light/base/image.h"
#include "modules/perception/traffic_light/base/light.h"

namespace apollo {
namespace perception {
namespace traffic_light {

struct RecognizeOption {};

// @brief Recognizer classify the light color.
class BaseRecognizer {
 public:
  BaseRecognizer() = default;

  virtual ~BaseRecognizer() = default;

  virtual bool Init() = 0;

  // @brief: recognize light status
  // @param [in] const Recognize&: recognize options
  // @param [in] const Image&: input image
  // @param [in/out] std::vector<Light>*: recognized light status
  // @return  bool
  virtual bool RecognizeStatus(const Image &image,
                               const RecognizeOption &option,
                               std::vector<LightPtr> *lights) = 0;

  virtual std::string name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseRecognizer);
};

REGISTER_REGISTERER(BaseRecognizer);
#define REGISTER_RECOGNIZER(name) REGISTER_CLASS(BaseRecognizer, name)

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_TRAFFIC_LIGHT_INTERFACE_BASE_RECOGNIZER_H
