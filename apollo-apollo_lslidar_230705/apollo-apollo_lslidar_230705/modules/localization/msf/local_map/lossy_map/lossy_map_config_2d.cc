/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/localization/msf/local_map/lossy_map/lossy_map_config_2d.h"

namespace apollo {
namespace localization {
namespace msf {

LossyMapConfig2D::LossyMapConfig2D(std::string map_version)
    : BaseMapConfig(map_version) {
  map_layer_alt_thres_ = 10000.0;  // in meters
  map_cache_size_ = 50;            // 80
  max_intensity_value_ = 255.0;
  max_intensity_var_value_ = 1000.0;
  map_is_compression_ = true;
  map_ground_height_offset_ = 1.7f;  // Set the initial value here.
}

void LossyMapConfig2D::CreateXml(boost::property_tree::ptree* config) const {
  BaseMapConfig::CreateXml(config);
  config->put("map.map_runtime.layer_alt_thres", map_layer_alt_thres_);
  config->put("map.map_runtime.cache_size", map_cache_size_);
  config->put("map.map_runtime.max_intensity_value", max_intensity_value_);
  config->put("map.map_runtime.max_intensity_var_value",
              max_intensity_var_value_);
}

void LossyMapConfig2D::LoadXml(const boost::property_tree::ptree& config) {
  BaseMapConfig::LoadXml(config);
  map_layer_alt_thres_ = config.get<float>("map.map_runtime.layer_alt_thres");
  map_cache_size_ = config.get<unsigned int>("map.map_runtime.cache_size");
  max_intensity_value_ =
      config.get<float>("map.map_runtime.max_intensity_value");
  max_intensity_var_value_ =
      config.get<float>("map.map_runtime.max_intensity_var_value");
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
