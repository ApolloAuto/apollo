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
#ifndef PERCEPTION_COLOR_DECISION_H
#define PERCEPTION_COLOR_DECISION_H

#include "modules/perception/traffic_light/interface/base_reviser.h"
#include "modules/perception/traffic_light/interface/green_interface.h"

namespace apollo {
namespace perception {
namespace traffic_light {

class ColorReviser : public BaseReviser {
 public:
  ColorReviser() {}

  //@brief init the reviser.
  virtual bool Init();
  //@brief reviser revise  the perception result
  //       ASSERT(rectifed_result.size == perception_result.size)
  //@param [in] option
  //@param [in/out] rectifed_result
  //@return true/false
  virtual bool Revise(const ReviseOption &option,
                      std::vector<LightPtr> *lights) override;

  //@brief Revise's name
  virtual std::string name() const;

 private:
  float blink_time_;
  int enable_;
  std::map<std::string, TLColor> color_map_;
  std::map<std::string, double> time_map_;
};
REGISTER_REVISER(ColorReviser);
}
}
}

#endif  // PERCEPTION_COLOR_DECISION_H
