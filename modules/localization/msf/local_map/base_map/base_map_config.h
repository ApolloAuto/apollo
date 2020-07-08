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
#include <vector>

#include <boost/property_tree/xml_parser.hpp>

#include "modules/localization/msf/common/util/rect2d.h"
#include "modules/localization/msf/local_map/base_map/base_map_fwd.h"

namespace apollo {
namespace localization {
namespace msf {

/**@brief The options of the reflectance map. */
class BaseMapConfig {
 public:
  /**@brief The constructor gives the default map settings. */
  explicit BaseMapConfig(std::string map_version = "0.1");
  /**@brief Save the map option to a XML file. */
  bool Save(const std::string file_path);
  /**@brief Load the map option from a XML file. */
  bool Load(const std::string file_path);
  /**@brief Resize map range by range and resolutions. */
  void ResizeMapRange();
  /**@brief Set single resolutions. */
  void SetSingleResolutions(float resolution = 0.125);
  /**@brief Set multi resolutions. */
  void SetMultiResolutions();

  /**@brief The version of map. */
  std::string map_version_;
  /**@brief The pixel resolutions in the map in meters. */
  std::vector<float> map_resolutions_;
  /**@brief The map node size in pixels. */
  unsigned int map_node_size_x_;
  /**@brief The map node size in pixels. */
  unsigned int map_node_size_y_;
  /**@brief The minimum and maximum UTM range in the map.
   *
   * The x direction is the easting in UTM coordinate.
   * The y direction is the northing in UTM coordinate.
   */
  Rect2D<double> map_range_;

  /**@brief Velodyne's height to the ground. Estimate the Velodyne's height
   * based on the ground height. */
  float map_ground_height_offset_;
  /**@brief Enable the compression. */
  bool map_is_compression_;

  /**@brief The map folder path. */
  std::string map_folder_path_;
  /**@brief The datasets that contributed to the map. */
  std::vector<std::string> map_datasets_;

 protected:
  /**@brief Create the XML structure. */
  virtual void CreateXml(boost::property_tree::ptree* config) const;
  /**@brief Load the map options from a XML structure. */
  virtual void LoadXml(const boost::property_tree::ptree& config);
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
