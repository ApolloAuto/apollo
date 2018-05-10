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

#include "modules/perception/traffic_light/reviser/color_decision.h"

#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/traffic_light/base/tl_shared_data.h"

namespace apollo {
namespace perception {
namespace traffic_light {

std::string ColorReviser::name() const { return "ColorReviser"; }
bool ColorReviser::Init() {
  ConfigManager *config_manager = ConfigManager::instance();
  if (config_manager == nullptr) {
    AERROR << "failed to get ConfigManager instance.";
    return false;
  }

  const ModelConfig *model_config = config_manager->GetModelConfig(name());
  if (model_config == nullptr) {
    AERROR << "not found model config: " << name();
    return false;
  }

  if (!model_config->GetValue("enable", &enable_)) {
    AERROR << "enable not found." << name();
    return false;
  }

  if (!model_config->GetValue("blink_time", &blink_time_)) {
    AERROR << "blink_time not found." << name();
    return false;
  }

  return true;
}
bool ColorReviser::Revise(const ReviseOption &option,
                          std::vector<LightPtr> *lights) {
  if (enable_ == 0) {
    return true;
  }
  std::vector<LightPtr> &lights_ref = *lights;

  for (size_t i = 0; i < lights_ref.size(); ++i) {
    std::string id = lights_ref[i]->info.id().id();
    ADEBUG << "light " << i << " id: " << id << std::endl;
    switch (lights_ref[i]->status.color) {
      default:
      case BLACK:
      case UNKNOWN_COLOR:
        if (color_map_.find(id) != color_map_.end() && option.ts > 0 &&
            option.ts - time_map_[id] < blink_time_) {
          AINFO << "Revise " << kColorStr[lights_ref[i]->status.color]
                << " to color " << kColorStr[color_map_[id]];
          lights_ref[i]->status.color = color_map_[id];
        } else {
          AINFO << "Unrevised color " << kColorStr[lights_ref[i]->status.color];
        }
        break;
      case YELLOW:
        // if YELLOW appears after RED, revise it to RED
        if (color_map_.find(id) != color_map_.end() && option.ts > 0 &&
            color_map_.at(id) == RED) {
          lights_ref[i]->status.color = color_map_.at(id);
          AINFO << "Revise Yellow to color Red";
          color_map_[id] = RED;
          time_map_[id] = option.ts;
          break;
        }
      case RED:
      case GREEN:
        if (time_map_.size() > 10) {
          color_map_.clear();
          time_map_.clear();
        }
        color_map_[id] = lights_ref[i]->status.color;
        time_map_[id] = option.ts;
        AINFO << "Revise Keep Color Unchanged: "
              << kColorStr[lights_ref[i]->status.color];
        break;
    }
  }

  return true;
}
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
