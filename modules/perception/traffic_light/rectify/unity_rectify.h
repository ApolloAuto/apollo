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
#ifndef MODULES_PERCEPTION_TRAFFIC_LIGHT_RECTIFY_UNITY_RECTIFY_H_
#define MODULES_PERCEPTION_TRAFFIC_LIGHT_RECTIFY_UNITY_RECTIFY_H_

#include <memory>
#include <string>
#include <vector>

#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/traffic_light/interface/base_rectifier.h"
#include "modules/perception/traffic_light/interface/green_interface.h"

namespace apollo {
namespace perception {
namespace traffic_light {

class UnityRectify : public BaseRectifier {
 public:
  UnityRectify() = default;

  bool Init() override;

  // @brief: rectify light region from image or part of it
  // @param [in] const Image&: input image
  // @param [in] const RectifyOptions&: rectify options
  // @param [in/out] Lights
  // @return  bool
  bool Rectify(const Image &image, const RectifyOption &option,
               std::vector<LightPtr> *lights) override;

  bool InitDetection(const ConfigManager *config_manager,
                     const ModelConfig *model_config,
                     std::shared_ptr<IRefine> *detection,
                     std::shared_ptr<IGetBox> *crop);

  // @brief name
  std::string name() const override;

 private:
  std::shared_ptr<ISelectLight> select_;
  std::shared_ptr<IRefine> detect_;
  std::shared_ptr<IGetBox> crop_;
};

REGISTER_RECTIFIER(UnityRectify);

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_TRAFFIC_LIGHT_RECTIFY_UNITY_RECTIFY_H_
