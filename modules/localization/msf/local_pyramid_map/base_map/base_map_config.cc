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

#include "modules/localization/msf/local_map/base_map/base_map_config.h"

#include <boost/foreach.hpp>
#include <exception>
#include <iostream>

namespace apollo {
namespace localization {
namespace msf {

BaseMapConfig::BaseMapConfig(const std::string &map_version) {
  map_version_ = map_version;
  coordinate_type_ = "UTM";
  map_resolutions_.push_back(0.125);
  map_node_size_x_ = 1024;                                   // in pixels
  map_node_size_y_ = 1024;                                   // in pixels
  map_range_ = Rect2D<double>(0, 0, 1000448.0, 10000384.0);  // in meters
  map_ground_height_offset_ = 1.7f;
  map_is_compression_ = true;
  map_folder_path_ = ".";
}

BaseMapConfig::~BaseMapConfig() {}

bool BaseMapConfig::Save(const std::string &file_path) {
  boost::property_tree::ptree config;
  bool success = CreateXml(&config);
  if (success) {
    boost::property_tree::write_xml(file_path, config);
    std::cerr << "Saved the map configuration to: " << file_path << "."
              << std::endl;
    return true;
  } else {
    return false;
  }
}

bool BaseMapConfig::Load(const std::string &file_path) {
  boost::property_tree::ptree config;
  boost::property_tree::read_xml(file_path, config);
  bool success = LoadXml(config);

  if (success) {
    std::cerr << "Loaded the map configuration from: " << file_path << "."
              << std::endl;
    return true;
  } else {
    return false;
  }
}

bool BaseMapConfig::CreateXml(boost::property_tree::ptree *config) const {
  config->put("map.map_config.version", map_version_);
  config->put("map.map_config.coordnate_type", coordinate_type_);
  config->put("map.map_config.node_size.x", map_node_size_x_);
  config->put("map.map_config.node_size.y", map_node_size_y_);
  config->put("map.map_config.range.min_x", map_range_.GetMinX());
  config->put("map.map_config.range.min_y", map_range_.GetMinY());
  config->put("map.map_config.range.max_x", map_range_.GetMaxX());
  config->put("map.map_config.range.max_y", map_range_.GetMaxY());
  config->put("map.map_config.compression", map_is_compression_);
  config->put("map.map_runtime.map_ground_height_offset",
              map_ground_height_offset_);
  for (size_t i = 0; i < map_resolutions_.size(); ++i) {
    config->add("map.map_config.resolutions.resolution", map_resolutions_[i]);
  }

  for (size_t i = 0; i < map_datasets_.size(); ++i) {
    config->add("map.map_record.datasets.dataset", map_datasets_[i]);
  }

  // added md5 check info
  std::map<std::string, std::string>::const_iterator iter;
  for (iter = node_md5_map_.begin(); iter != node_md5_map_.end(); ++iter) {
    boost::property_tree::ptree child;
    child.put("path", iter->first);
    child.put("md5", iter->second);
    config->add_child("map.check_info.nodes.node", child);
  }

  return true;
}

bool BaseMapConfig::LoadXml(const boost::property_tree::ptree &config) {
  map_resolutions_.clear();
  map_datasets_.clear();
  node_md5_map_.clear();

  auto map_version = config.get_optional<std::string>("map.map_config.version");
  if (map_version) {
    map_version_ = map_version.get();
  }
  auto coordinate_type =
      config.get_optional<std::string>("map.map_config.coordinate_type");
  if (coordinate_type) {
    coordinate_type_ = coordinate_type.get();
  }
  auto map_node_size_x =
      config.get_optional<unsigned int>("map.map_config.node_size.x");
  if (map_node_size_x) {
    map_node_size_x_ = map_node_size_x.get();
  }

  auto map_node_size_y =
      config.get_optional<unsigned int>("map.map_config.node_size.y");
  if (map_node_size_y) {
    map_node_size_y_ = map_node_size_y.get();
  }
  double min_x = 0.0;
  auto tmp_min_x = config.get_optional<double>("map.map_config.range.min_x");
  if (tmp_min_x) {
    min_x = tmp_min_x.get();
  }
  double min_y = 0.0;
  auto tmp_min_y = config.get_optional<double>("map.map_config.range.min_y");
  if (tmp_min_y) {
    min_y = tmp_min_y.get();
  }
  double max_x = 0.0;
  auto tmp_max_x = config.get_optional<double>("map.map_config.range.max_x");
  if (tmp_max_x) {
    max_x = tmp_max_x.get();
  }
  double max_y = 0.0;
  auto tmp_max_y = config.get_optional<double>("map.map_config.range.max_y");
  if (tmp_max_y) {
    max_y = tmp_max_y.get();
  }
  map_range_ = Rect2D<double>(min_x, min_y, max_x, max_y);
  auto map_ground_height_offset =
      config.get_optional<float>("map.map_runtime.map_ground_height_offset");
  if (map_ground_height_offset) {
    map_ground_height_offset_ = map_ground_height_offset.get();
  }
  auto map_is_compression =
      config.get_optional<bool>("map.map_config.compression");
  if (map_is_compression) {
    map_is_compression_ = map_is_compression.get();
  }

  auto resolutions = config.get_child_optional("map.map_config.resolutions");
  if (resolutions) {
    BOOST_FOREACH (const boost::property_tree::ptree::value_type &v,
                   *resolutions) {
      map_resolutions_.push_back(
          static_cast<float>(atof(v.second.data().c_str())));
      AINFO << "Resolution: " << v.second.data();
    }
  } else {
    return false;
  }

  auto datasets = config.get_child_optional("map.map_record.datasets");
  if (datasets) {
    BOOST_FOREACH (const boost::property_tree::ptree::value_type &v,
                   *datasets) {
      map_datasets_.push_back(v.second.data());
      AINFO << "Dataset: " << v.second.data();
    }
  }

  // load md5 check info
  auto nodes = config.get_child_optional("map.check_info.nodes");
  if (nodes) {
    BOOST_FOREACH (const boost::property_tree::ptree::value_type &v, *nodes) {
      const boost::property_tree::ptree &child = v.second;
      auto path = child.get_optional<std::string>("path");
      auto md5 = child.get_optional<std::string>("md5");
      if (!path || !md5) {
        std::cerr << "Lack path or md5." << std::endl;
        return false;
      }

      node_md5_map_[*path] = *md5;
    }
  }

  return true;
}

void BaseMapConfig::SetMapVersion(const std::string &map_version) {
  map_version_ = map_version;
}

void BaseMapConfig::ResizeMapRange() {
  double min_x = 0;
  double min_y = 0;
  double max_x = 0;
  double max_y = 0;

  double max_resolutions = 0.0;
  double max_resolutions =
      std::max_element(map_resolutions_.begin(), map_resolutions_.end());
  int n = 0;
  while (min_x >= map_range_.GetMinX()) {
    ++n;
    min_x -= n * map_node_size_x_ * max_resolutions;
  }
  n = 0;
  while (min_y >= map_range_.GetMinY()) {
    ++n;
    min_y -= n * map_node_size_y_ * max_resolutions;
  }
  n = 0;
  while (max_x <= map_range_.GetMaxX()) {
    ++n;
    max_x += n * map_node_size_x_ * max_resolutions;
  }
  n = 0;
  while (max_y <= map_range_.GetMaxY()) {
    ++n;
    max_y += n * map_node_size_y_ * max_resolutions;
  }
  map_range_ = Rect2D<double>(min_x, min_y, max_x, max_y);
}

void BaseMapConfig::SetSingleResolutions(float resolution) {
  map_resolutions_.clear();
  map_resolutions_.push_back(resolution);
}

void BaseMapConfig::SetMultiResolutions() {
  map_resolutions_.clear();
  map_resolutions_.push_back(0.03125);
  map_resolutions_.push_back(0.0625);
  map_resolutions_.push_back(0.125);
  map_resolutions_.push_back(0.25);
  map_resolutions_.push_back(0.5);
  map_resolutions_.push_back(1);
  map_resolutions_.push_back(2);
  map_resolutions_.push_back(4);
  map_resolutions_.push_back(8);
  map_resolutions_.push_back(16);
}

void BaseMapConfig::SetNodeMd5Map(
    const std::map<std::string, std::string> &node_md5_map) {
  node_md5_map_ = node_md5_map;
}

void BaseMapConfig::AddNodeMd5(const std::string &node_path,
                               const std::string &md5) {
  node_md5_map_[node_path] = md5;
}

void BaseMapConfig::SetMapNodeSize(unsigned int size_x, unsigned int size_y) {
  map_node_size_x_ = size_x;
  map_node_size_y_ = size_y;
}

void BaseMapConfig::SetGroundHeightOffset(float map_ground_height_offset) {
  map_ground_height_offset_ = map_ground_height_offset;
}

void BaseMapConfig::SetIsCompression(bool map_is_compression) {
  map_is_compression_ = map_is_compression;
}

MapVersion BaseMapConfig::GetMapVersion() const {
  if (map_version_ == "lossy_full_alt" || map_version_ == "lossy_map") {
    return MapVersion::LOSSY_FULL_ALT_MAP;
  }

  if (map_version_ == "0.21" || map_version_ == "lossless_map") {
    return MapVersion::LOSSLESS_MAP;
  }

  if (map_version_ == "pyramid_lossy_map") {
    return MapVersion::PYRAMID_LOSSY_MAP;
  }

  if (map_version_ == "pyramid_lossless_map") {
    return MapVersion::PYRAMID_LOSSLESS_MAP;
  }

  return MapVersion::UNKNOWN;
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
