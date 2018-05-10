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
#ifndef MODULES_PERCEPTION_TRFFIC_LIGHT_REVISER_COLOR_DECISION_H_
#define MODULES_PERCEPTION_TRFFIC_LIGHT_REVISER_COLOR_DECISION_H_

#include <string>
#include <unordered_map>
#include <vector>

#include "modules/perception/traffic_light/interface/base_reviser.h"
#include "modules/perception/traffic_light/interface/green_interface.h"

namespace apollo {
namespace perception {
namespace traffic_light {

/**
 * @class BaseReviser
 * @brief Reviser is the class is to revise the perception result.
 *        It may use history info or some else info.
 */
class ColorReviser : public BaseReviser {
 public:
  ColorReviser() = default;

  /**
   * @brief init the reviser.
   */
  virtual bool Init();

  /**
   * @brief reviser revise  the perception result
   *       ASSERT(rectifed_result.size == perception_result.size)
   * @param option
   * @param rectifed_result
   * @return true/false
   */
  bool Revise(const ReviseOption &option,
              std::vector<LightPtr> *lights) override;

  /**
   * @brief Revise's name
   */
  std::string name() const override;

 private:
  float blink_time_ = 0.0;
  int enable_ = 0.0;
  std::unordered_map<std::string, TLColor> color_map_;
  std::unordered_map<std::string, double> time_map_;
};
REGISTER_REVISER(ColorReviser);

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_TRAFFIC_LIGHT_REVISER_COLOR_DECISION_H_
