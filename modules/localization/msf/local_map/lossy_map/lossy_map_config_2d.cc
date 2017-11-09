#include "modules/localization/msf/local_map/lossy_map/lossy_map_config_2d.h"

namespace apollo {
namespace localization {
namespace msf {

LossyMapConfig2D::LossyMapConfig2D(std::string map_version)
    : BaseMapConfig(map_version) {
  _map_layer_alt_thres = 10000.0;  // in meters
  _map_cache_size = 50;            // 80
  _max_intensity_value = 255.0;
  _max_intensity_var_value = 1000.0;
  _map_is_compression = true;
  _map_ground_height_offset = 1.7;  // Set the initial value here.
}

void LossyMapConfig2D::create_xml(boost::property_tree::ptree& config) const {
  BaseMapConfig::create_xml(config);
  config.put("map.map_runtime.layer_alt_thres", _map_layer_alt_thres);
  config.put("map.map_runtime.cache_size", _map_cache_size);
  config.put("map.map_runtime.max_intensity_value", _max_intensity_value);
  config.put("map.map_runtime.max_intensity_var_value",
             _max_intensity_var_value);
  return;
}

void LossyMapConfig2D::load_xml(boost::property_tree::ptree& config) {
  BaseMapConfig::load_xml(config);
  _map_layer_alt_thres = config.get<float>("map.map_runtime.layer_alt_thres");
  _map_cache_size = config.get<unsigned int>("map.map_runtime.cache_size");
  _max_intensity_value =
      config.get<float>("map.map_runtime.max_intensity_value");
  _max_intensity_var_value =
      config.get<float>("map.map_runtime.max_intensity_var_value");
  return;
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
