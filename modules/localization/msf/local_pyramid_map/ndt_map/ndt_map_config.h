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

#pragma once

#include <string>
#include <vector>
#include "modules/localization/msf/local_map/base_map/base_map_config.h"

namespace apollo {
namespace localization {
namespace msf {

/**@brief The options of the reflectance map. */
class NdtMapConfig : public BaseMapConfig {
 public:
  /**@brief The constructor gives the default map settings. */
  explicit NdtMapConfig(std::string map_version = "0.1");
  ~NdtMapConfig() {}

  /**@brief Set single resolutions. */
  void SetSingleResolutionZ(float resolution = 1.0f);
  /**@brief Set multi resolutions. */
  void SetMultiResolutionsZ();

  /**@brief The resolution of z-axis. */
  std::vector<float> map_resolutions_z_;

  /**@brief Enable the compression. */
  bool map_is_compression_;

 protected:
  /**@brief Create the XML structure. */
  virtual bool CreateXml(boost::property_tree::ptree* config) const;
  /**@brief Load the map options from a XML structure. */
  virtual bool LoadXml(boost::property_tree::ptree* config);
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
