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

#include <iostream>
#include <string>

#include "modules/localization/msf/local_pyramid_map/base_map/base_map_config.h"
#include "modules/localization/msf/local_pyramid_map/base_map/base_map_fwd.h"

namespace apollo {
namespace localization {
namespace msf {
namespace pyramid_map {

class MapNodeIndex;

std::ostream& operator<<(std::ostream& cout, const MapNodeIndex& index);

class MapNodeIndex {
 public:
  /**@brief The constructor. */
  MapNodeIndex();
  /**@brief Overload the less than operator. */
  bool operator<(const MapNodeIndex& index) const;
  /**@brief Overload the equal operator. */
  bool operator==(const MapNodeIndex& index) const;
  /**@brief Overload the unequal operator. */
  bool operator!=(const MapNodeIndex& index) const;
  std::string ToString() const;

  /**@brief Construct a map node index, given a global coordinate, eigen
   * version. */
  static MapNodeIndex GetMapNodeIndex(const BaseMapConfig& option,
                                      const Eigen::Vector3d& coordinate,
                                      unsigned int resolution_id, int zone_id);
  static MapNodeIndex GetMapNodeIndex(const BaseMapConfig& option,
                                      const Eigen::Vector2d& coordinate,
                                      unsigned int resolution_id, int zone_id);

  /**@brief Get the index range (maximum possible index + 1) in the east
   * direction. */
  static unsigned int GetMapIndexRangeEast(const BaseMapConfig& option,
                                           unsigned int resolution_id);
  /**@brief Get the index range (maximum possible index + 1) in the north
   * direction. */
  static unsigned int GetMapIndexRangeNorth(const BaseMapConfig& option,
                                            unsigned int resolution_id);

  friend std::ostream& operator<<(std::ostream& cout,
                                  const MapNodeIndex& index);

  /**@brief The ID of the resolution.
   * Should be less than BaseMapConfig::_map_resolutions.size(). */
  unsigned int resolution_id_ = 0;
  /**@brief The zone ID. 1 - 60 and -1 - -60.
   * The positive value is the zone at the north hemisphere. */
  int zone_id_ = 10;
  /**@brief The map node ID at the northing direction. */
  unsigned int m_ = 0;
  /**@brief The map node ID at the easting direction. */
  unsigned int n_ = 0;
};

}  // namespace pyramid_map
}  // namespace msf
}  // namespace localization
}  // namespace apollo
