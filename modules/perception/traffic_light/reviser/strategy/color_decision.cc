//
// Created by gaohan02 on 16-12-2.
//

#include <modules/perception/lib/config_manager/config_manager.h>
#include "color_decision.h"

namespace apollo {
namespace perception {
namespace traffic_light {

std::map<TLColor, std::string> ColorReviser::_s_color_strs = {
    {UNKNOWN_COLOR, "unknown"},
    {RED, "red"},
    {GREEN, "green"},
    {YELLOW, "yellow"},
    {BLACK, "black"}
};

std::string ColorReviser::name() const {
  return "ColorReviser";
}
bool ColorReviser::init() {
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
            "enable", &_enable)) {
    AERROR << "enable not found." << name();
    return false;
  }

  if (!model_config->GetValue(\
            "blink_time", &_blink_time)) {
    AERROR << "blink_time not found." << name();
    return false;
  }

  return true;
}
bool ColorReviser::revise(const ReviseOption &option, std::vector<LightPtr> *lights) {

  if (_enable == 0) {
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
        if (_color_map.find(id) != _color_map.end() &&
            option.ts > 0 && option.ts - _time_map[id] < _blink_time) {
          AINFO << "Revise " << _s_color_strs[lights_ref[i]->status.color]
                << " to color " << _s_color_strs[_color_map[id]];
          lights_ref[i]->status.color = _color_map[id];
        } else {
          AINFO << "Unrevised color " << _s_color_strs[lights_ref[i]->status.color];
        }
        break;
      case YELLOW:
        // if YELLOW appears after RED, revise it to RED
        if (_color_map.find(id) != _color_map.end() &&
            option.ts > 0 && _color_map.at(id) == RED) {
          lights_ref[i]->status.color = _color_map.at(id);
          AINFO << "Revise Yellow to color Red";
          _color_map[id] = RED;
          _time_map[id] = option.ts;
          break;
        }
      case RED:
      case GREEN:
        if (_time_map.size() > 10) {
          _color_map.clear();
          _time_map.clear();
        }
        _color_map[id] = lights_ref[i]->status.color;
        _time_map[id] = option.ts;
        AINFO << "Revise Keep Color Unchanged: "
              << _s_color_strs[lights_ref[i]->status.color];
        break;
    }
  }

  return true;
}

REGISTER_REVISER(ColorReviser);

}
}
}
