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

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <map>
#include <string>
#include <vector>
#include "modules/localization/msf/common/util/rect2d.h"
#include "modules/localization/msf/local_map/base_map/base_map_fwd.h"

namespace apollo {
namespace localization {
namespace msf {

enum class MapVersion {
  UNKNOWN,
  LOSSY_FULL_ALT_MAP,
  LOSSLESS_MAP,
  PYRAMID_LOSSY_MAP,
  PYRAMID_LOSSLESS_MAP
};

/**@brief The options of the reflectance map. */
class BaseMapConfig {
 public:
  /**@brief The constructor gives the default map settings. */
  explicit BaseMapConfig(const std::string &map_version = "0.1");
  /**@brief The deconstructor. */
  virtual ~BaseMapConfig();
  /**@brief Save the map option to a XML file. */
  bool Save(const std::string &file_path);
  /**@brief Load the map option from a XML file. */
  bool Load(const std::string &file_path);
  /**@brief Set map_version. */
  void SetMapVersion(const std::string &map_version);
  /**@brief Set map_node_size. */
  void SetMapNodeSize(unsigned int size_x, unsigned int size_y);
  /**@brief Set map_ground_height_offset. */
  void SetGroundHeightOffset(float map_ground_height_offset);
  /**@brief Set map_is_compression. */
  void SetIsCompression(bool map_is_compression);
  /**@brief Resize map range by range and resolutions. */
  void ResizeMapRange();
  /**@brief Set single resolutions. */
  void SetSingleResolutions(float resolution = 0.125);
  /**@brief Set multi resolutions. */
  void SetMultiResolutions();
  /**@brief Set node_md5_map. */
  void SetNodeMd5Map(const std::map<std::string, std::string> &node_md5_map);
  /**@brief Add a node md5 pair. */
  void AddNodeMd5(const std::string &node_path, const std::string &md5);
  /**@brief Get map version. */
  MapVersion GetMapVersion() const;

  /**@brief The version of map. */
  std::string map_version_ = "";
  /**@brief The pixel resolutions in the map in meters. */
  std::vector<float> map_resolutions_;
  /**@brief The map node size in pixels. */
  unsigned int map_node_size_x_ = 0;
  /**@brief The map node size in pixels. */
  unsigned int map_node_size_y_ = 0;
  /**@brief The minimum and maximum UTM range in the map.
   * The x direction is the easting in UTM coordinate.
   * The y direction is the northing in UTM coordinate.
   */
  Rect2D<double> map_range_;

  /**@brief Velodyne's height to the ground. Estimate the Velodyne's height
   * based on the ground height. */
  float map_ground_height_offset_ = 0.0f;
  /**@brief Enable the compression. */
  bool map_is_compression_ = false;

  /**@brief The map folder path. */
  std::string map_folder_path_ = "";
  /**@brief The datasets that contributed to the map. */
  std::vector<std::string> map_datasets_;
  /**@brief The map structure to store map node file name and its md5.
   * key：map node file name; value: md5 of map node file. */
  std::map<std::string, std::string> node_md5_map_;

  std::string coordinate_type_ = "";

 protected:
  /**@brief Create the XML structure. */
  virtual bool CreateXml(boost::property_tree::ptree *config) const;
  /**@brief Load the map options from a XML structure. */
  virtual bool LoadXml(const boost::property_tree::ptree &config);
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
