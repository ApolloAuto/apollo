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
#ifndef MODULES_PERCEPTION_TRAFFIC_LIGHT_RECOGNIZE_UNITY_RECOGNIZE_H
#define MODULES_PERCEPTION_TRAFFIC_LIGHT_RECOGNIZE_UNITY_RECOGNIZE_H

#include <memory>
#include <string>
#include <vector>
#include "modules/perception/traffic_light/interface/base_recognizer.h"
#include "modules/perception/traffic_light/interface/green_interface.h"
#include "modules/perception/lib/config_manager/config_manager.h"

namespace apollo {
namespace perception {
namespace traffic_light {
class UnityRecognize : public BaseRecognizer {
 public:
  UnityRecognize() = default;

  bool Init() override;

  // @brief: recognize light status
  // @param [in] const Recognize&: recognize options
  // @param [in] const Image&: input image
  // @param [in/out] std::vector<Light>*: recognized light status
  // @return  bool
  bool RecognizeStatus(const Image &image, const RecognizeOption &option,
                       std::vector<LightPtr> *lights) override;

  virtual std::string name() const;

 private:
  std::shared_ptr<IRefine> classify_day_;
  std::shared_ptr<IRefine> classify_night_;

  bool InitModel(const apollo::perception::ConfigManager *config_manager,
                 const apollo::perception::ModelConfig *model_config,
                 std::shared_ptr<IRefine> *classify);
};

REGISTER_RECOGNIZER(UnityRecognize);
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
#endif
