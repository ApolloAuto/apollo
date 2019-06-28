/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
#include "modules/localization/msf/local_map/pyramid_map/pyramid_map_config.h"

#include <string>

namespace apollo {
namespace localization {
namespace msf {

PyramidMapConfig::PyramidMapConfig(const std::string& map_version)
    : BaseMapConfig(map_version) {}

PyramidMapConfig::~PyramidMapConfig() {}

bool PyramidMapConfig::CreateXml(boost::property_tree::ptree* config) const {
  bool success = BaseMapConfig::CreateXml(config);
  if (success) {
    config->put("map.map_config.has_intensity", has_intensity_);
    config->put("map.map_config.has_intensity_var", has_intensity_var_);
    config->put("map.map_config.has_altitude", has_altitude_);
    config->put("map.map_config.has_altitude_var", has_altitude_var_);
    config->put("map.map_config.has_ground_altitude", has_ground_altitude_);
    config->put("map.map_config.has_count", has_count_);
    config->put("map.map_config.has_ground_count", has_ground_count_);
    config->put("map.map_config.resolution_num", resolution_num_);
    config->put("map.map_config.resolution_ratio", resolution_ratio_);
    config->put("map.map_config.coordinate_type", coordinate_type_);
    config->put("map.map_runtime.max_intensity_value", max_intensity_value_);
    config->put("map.map_runtime.max_intensity_var_value",
                max_intensity_var_value_);
    return true;
  } else {
    return false;
  }
}

bool PyramidMapConfig::LoadXml(const boost::property_tree::ptree& config) {
  bool success = BaseMapConfig::LoadXml(config);
  if (success) {
    auto has_intensity =
        config.get_optional<bool>("map.map_config.has_intensity");
    auto has_intensity_var =
        config.get_optional<bool>("map.map_config.has_intensity_var");
    auto has_altitude =
        config.get_optional<bool>("map.map_config.has_altitude");
    auto has_altitude_var =
        config.get_optional<bool>("map.map_config.has_altitude_var");
    auto has_ground_altitude =
        config.get_optional<bool>("map.map_config.has_ground_altitude");
    auto has_count = config.get_optional<bool>("map.map_config.has_count");
    auto has_ground_count =
        config.get_optional<bool>("map.map_config.has_ground_count");
    auto resolution_num =
        config.get_optional<unsigned int>("map.map_config.resolution_num");
    auto resolution_ratio =
        config.get_optional<unsigned int>("map.map_config.resolution_ratio");
    auto coordinate_type =
        config.get_optional<std::string>("map.map_config.coordinate_type");
    auto max_intensity_value =
        config.get_optional<float>("map.map_runtime.max_intensity_value");
    auto max_intensity_var_value =
        config.get_optional<float>("map.map_runtime.max_intensity_var_value");

    if (has_intensity) {
      has_intensity_ = *has_intensity;
    }

    if (has_intensity_var) {
      has_intensity_var_ = *has_intensity_var;
    }

    if (has_altitude) {
      has_altitude_ = *has_altitude;
    }

    if (has_altitude_var) {
      has_altitude_var_ = *has_altitude_var;
    }

    if (has_ground_altitude) {
      has_ground_altitude_ = *has_ground_altitude;
    }

    if (has_count) {
      has_count_ = *has_count;
    }

    if (has_ground_count) {
      has_ground_count_ = *has_ground_count;
    }

    if (resolution_num) {
      resolution_num_ = *resolution_num;
    }

    if (resolution_ratio) {
      resolution_ratio_ = *resolution_ratio;
    }

    if (coordinate_type) {
      coordinate_type_ = *coordinate_type;
    }

    if (max_intensity_value) {
      max_intensity_value_ = *max_intensity_value;
    }

    if (max_intensity_var_value) {
      max_intensity_var_value_ = *max_intensity_var_value;
    }

    return true;
  }

  return false;
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
