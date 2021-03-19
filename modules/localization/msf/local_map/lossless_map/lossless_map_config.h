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

#pragma once

#include <string>

#include "modules/localization/msf/local_map/base_map/base_map_config.h"

namespace apollo {
namespace localization {
namespace msf {

/**@brief The options of the reflectance map. */
class LosslessMapConfig : public BaseMapConfig {
 public:
  /**@brief The constructor gives the default map settings. */
  explicit LosslessMapConfig(std::string map_version = "lossless_map");
  ~LosslessMapConfig() {}

  /**@brief The threshold to split more layers in the map node. */
  float map_layer_alt_thres_;
  /**@brief When load map nodes, the maximum number of map nodes will be kept in
   * memory. */
  unsigned int map_cache_size_;
  /**@brief coordinate type. */
  std::string coordinate_type_;

  /**@brief During the visualization (for example, call the function get_image()
   * of map node layer), the maximum intensity value in the image. */
  float max_intensity_value_;
  /**@brief During the visualization (for example, call the function get_image()
   * of map node layer), the maximum intensity variance value in the image. */
  float max_intensity_var_value_;

 protected:
  /**@brief Create the XML structure. */
  virtual void CreateXml(boost::property_tree::ptree* config) const;
  /**@brief Load the map options from a XML structure. */
  virtual void LoadXml(const boost::property_tree::ptree& config);
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
