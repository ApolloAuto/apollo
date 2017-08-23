/* Copyright 2017 The Apollo Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
=========================================================================*/

#ifndef MODULES_MAP_HDMAP_HDMAP_UTIL_H
#define MODULES_MAP_HDMAP_HDMAP_UTIL_H

#include <memory>
#include <mutex>
#include <string>

#include "modules/common/configs/config_gflags.h"
#include "modules/common/util/string_util.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/proto/map_id.pb.h"

/**
 * @namespace apollo::hdmap
 * @brief apollo::hdmap
 */
namespace apollo {
namespace hdmap {

/**
 * @brief get base map file path from flags.
 * @return base map path
 */
std::string BaseMapFile();

/**
 * @brief get simulation map file path from flags.
 * @return simulation map path
 */
std::string SimMapFile();

/**
 * @brief get routing map file path from flags.
 * @return routing map path
 */
std::string RoutingMapFile();

/**
 * @brief get end way point file path from flags.
 * @return end way point file path
 */
inline std::string EndWayPointFile() {
  return apollo::common::util::StrCat(
      FLAGS_map_dir, "/", FLAGS_end_way_point_filename);
}

/**
 * @brief create a Map ID given a string.
 * @param id a string id
 * @return a Map ID instance
 */
inline apollo::hdmap::Id MakeMapId(const std::string& id) {
  apollo::hdmap::Id map_id;
  map_id.set_id(id);
  return map_id;
}

std::unique_ptr<HDMap> CreateMap(const std::string& map_file_path);

class HDMapUtil {
 public:
  // Get default base map from the file specified by global flags.
  // Return nullptr if failed to load.
  const HDMap* BaseMap();
  // Similar to BaseMap(), but garantee to return a valid HDMap, or else raise
  // fatal error.
  const HDMap& BaseMapRef();

  // Reload the base map from the file specified by global flags.
  bool ReloadBaseMap();

 private:
  std::unique_ptr<HDMap> base_map_ = nullptr;
  std::mutex base_map_mutex_;

  DECLARE_SINGLETON(HDMapUtil);
};

}  // namespace hdmap
}  // namespace apollo

#endif  // MODULES_MAP_HDMAP_HDMAP_UTIL_H
