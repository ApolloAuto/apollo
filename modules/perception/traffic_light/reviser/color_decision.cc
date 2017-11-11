//
// Created by gaohan02 on 16-12-2.
//

#include "color_decision.h"
#include "modules/perception/lib/config_manager/config_manager.h"

namespace apollo {
namespace perception {
namespace traffic_light {

std::map<TLColor, std::string> ColorReviser::s_color_strs_ = {
    {UNKNOWN_COLOR, "unknown"},
    {RED, "red"},
    {GREEN, "green"},
    {YELLOW, "yellow"},
    {BLACK, "black"}
};

std::string ColorReviser::name() const {
  return "ColorReviser";
}
bool ColorReviser::Init() {
  ConfigManager *config_manager = ConfigManager::instance();
  if (config_manager == NULL) {
    AERROR << "failed to get ConfigManager instance.";
    return false;
  }

  const ModelConfig *model_config = NULL;
  if (!config_manager->GetModelConfig(name(), &model_config)) {
    AERROR << "not found model config: " << name();
    return false;
  }

  if (!model_config->GetValue(\
            "enable", &enable_)) {
    AERROR << "enable not found." << name();
    return false;
  }

  if (!model_config->GetValue(\
            "blink_time", &blink_time_)) {
    AERROR << "blink_time not found." << name();
    return false;
  }

  return true;
}
bool ColorReviser::Revise(const ReviseOption &option, std::vector<LightPtr> *lights) {

  if (enable_ == 0) {
    return true;
  }
  std::vector<LightPtr> &lights_ref = *lights;

  for (int i = 0; i < lights_ref.size(); ++i) {
    std::string id = lights_ref[i]->info.id().id();
    std::cout << "light " << i << " id: " << id << std::endl;
    switch (lights_ref[i]->status.color) {
      default:
      case BLACK:
      case UNKNOWN_COLOR:
        if (color_map_.find(id) != color_map_.end() &&
            option.ts > 0 && option.ts - time_map_[id] < blink_time_) {
          AINFO << "Revise " << s_color_strs_[lights_ref[i]->status.color]
                << " to color " << s_color_strs_[color_map_[id]];
          lights_ref[i]->status.color = color_map_[id];
        } else {
          AINFO << "Unrevised color " << s_color_strs_[lights_ref[i]->status.color];
        }
        break;
      case YELLOW:
        // if YELLOW appears after RED, revise it to RED
        if (color_map_.find(id) != color_map_.end() &&
            option.ts > 0 && color_map_.at(id) == RED) {
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
              << s_color_strs_[lights_ref[i]->status.color];
        break;
    }
  }

  return true;
}

}
}
}
