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

#ifndef MODULES_MAP_HDMAP_HDMAP_UTIL_H_
#define MODULES_MAP_HDMAP_HDMAP_UTIL_H_

#include <memory>
#include <mutex>
#include <string>

#include "modules/map/proto/map_id.pb.h"
#include "modules/map/proto/map_speed_control.pb.h"

#include "modules/common/configs/config_gflags.h"
#include "modules/common/util/file.h"
#include "modules/common/util/string_util.h"
#include "modules/map/hdmap/hdmap.h"

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
  if (FLAGS_use_navigation_mode) {
    return apollo::common::util::StrCat(
        FLAGS_navigation_mode_end_way_point_file);
  } else {
    return apollo::common::util::StrCat(FLAGS_map_dir, "/",
                                        FLAGS_end_way_point_filename);
  }
}

inline std::string SpeedControlFile() {
  return apollo::common::util::StrCat(FLAGS_map_dir, "/",
                                      FLAGS_speed_control_filename);
}

const SpeedControls* GetSpeedControls();

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
  static const HDMap* BaseMapPtr();
  // Guarantee to return a valid base_map, or else raise fatal error.
  static const HDMap& BaseMap();

  // Get default sim_map from the file specified by global flags.
  // Return nullptr if failed to load.
  static const HDMap* SimMapPtr();

  // Guarantee to return a valid sim_map, or else raise fatal error.
  static const HDMap& SimMap();

  // Reload maps from the file specified by global flags.
  static bool ReloadMaps();

 private:
  HDMapUtil() = delete;

  static std::unique_ptr<HDMap> base_map_;
  static uint64_t base_map_seq_;
  static std::mutex base_map_mutex_;

  static std::unique_ptr<HDMap> sim_map_;
  static std::mutex sim_map_mutex_;
};

}  // namespace hdmap
}  // namespace apollo

#endif  // MODULES_MAP_HDMAP_HDMAP_UTIL_H_
