//
// Created by gaohan02 on 16-12-2.
//

#include <lib/config_manager/config_manager.h>
#include "color_decision.h"

namespace adu {
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
  config_manager::ConfigManager *config_manager = \
            base::Singleton<config_manager::ConfigManager>::get();
  if (config_manager == NULL) {
    AERROR << "failed to get ConfigManager instance.";
    return false;
  }

  const config_manager::ModelConfig *model_config = NULL;
  if (!config_manager->get_model_config(name(), &model_config)) {
    AERROR << "not found model config: " << name();
    return false;
  }

  if (!model_config->get_value(\
            "enable", &_enable)) {
    AERROR << "enable not found." << name();
    return false;
  }

  if (!model_config->get_value(\
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
          lights_ref[i]->status.color = _color_map[id];
          AINFO << "Revise " << _s_color_strs[lights_ref[i]->status.color]
                << " to color " << _color_map[id];
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
