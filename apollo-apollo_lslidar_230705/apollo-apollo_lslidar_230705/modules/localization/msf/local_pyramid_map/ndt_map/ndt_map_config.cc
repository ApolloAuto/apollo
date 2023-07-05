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

#include "modules/localization/msf/local_pyramid_map/ndt_map/ndt_map_config.h"

#include <algorithm>
#include <string>

namespace apollo {
namespace localization {
namespace msf {
namespace pyramid_map {

NdtMapConfig::NdtMapConfig(std::string map_version)
    : BaseMapConfig(map_version) {
  map_is_compression_ = true;
  map_resolutions_z_.push_back(1.0f);
}

void NdtMapConfig::SetSingleResolutionZ(float resolution) {
  map_resolutions_z_.clear();
  map_resolutions_z_.push_back(resolution);
}

void NdtMapConfig::SetMultiResolutionsZ() {
  map_resolutions_z_.clear();
  map_resolutions_z_.push_back(0.03125);
  map_resolutions_z_.push_back(0.0625);
  map_resolutions_z_.push_back(0.125);
  map_resolutions_z_.push_back(0.25);
  map_resolutions_z_.push_back(0.5);
  map_resolutions_z_.push_back(1);
  map_resolutions_z_.push_back(2);
  map_resolutions_z_.push_back(4);
  map_resolutions_z_.push_back(8);
  map_resolutions_z_.push_back(16);
}

bool NdtMapConfig::CreateXml(boost::property_tree::ptree* config) const {
  BaseMapConfig::CreateXml(config);
  config->put("map.map_config.compression", map_is_compression_);
  for (size_t i = 0; i < map_resolutions_.size(); ++i) {
    config->add("map.map_config.resolutions_z.resolution",
                map_resolutions_z_[i]);
  }
  return true;
}

bool NdtMapConfig::LoadXml(boost::property_tree::ptree* config) {
  BaseMapConfig::LoadXml(*config);
  map_is_compression_ = config->get<bool>("map.map_config.compression");
  map_resolutions_z_.clear();
  const auto& resolutions_z = config->get_child("map.map_config.resolutions_z");
  std::for_each(resolutions_z.begin(), resolutions_z.end(),
                [this](const boost::property_tree::ptree::value_type& v) {
                  map_resolutions_z_.push_back(
                      static_cast<float>(atof(v.second.data().c_str())));
                });
  return true;
}

}  // namespace pyramid_map
}  // namespace msf
}  // namespace localization
}  // namespace apollo
