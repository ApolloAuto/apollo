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

#include "modules/localization/msf/local_pyramid_map/base_map/base_map.h"
#include "modules/localization/msf/local_pyramid_map/pyramid_map/pyramid_map_config.h"

namespace apollo {
namespace localization {
namespace msf {
namespace pyramid_map {

class PyramidMap : public BaseMap {
 public:
  explicit PyramidMap(PyramidMapConfig* config);
  ~PyramidMap();

 public:
  /**@brief Given the 3D global coordinate, this function loads the
   * corresponding map node in the cache thread safely and return intensity, if
   *necessary.
   **/
  float GetIntensitySafe(const Eigen::Vector3d& coordinate, int zone_id,
                         unsigned int resolution_id, unsigned int level = 0);
  /**@brief Given the 3D global coordinate, this function loads the
   * corresponding map node in the cache and return intensity var, if necessary.
   **/
  float GetIntensityVarSafe(const Vector3d& coordinate, int zone_id,
                            unsigned int resolution_id, unsigned int level = 0);
  /**@brief Given the 3D global coordinate, this function loads the
   * corresponding map node in the cache thread safely and return altitude, if
   *necessary.
   **/
  float GetAltitudeSafe(const Eigen::Vector3d& coordinate, int zone_id,
                        unsigned int resolution_id, unsigned int level = 0);
  /**@brief Given the 3D global coordinate, this function loads the
   * corresponding map node in the cache thread safely and return altitude var,
   *if necessary.
   **/
  float GetAltitudeVarSafe(const Eigen::Vector3d& coordinate, int zone_id,
                           unsigned int resolution_id, unsigned int level = 0);
  /**@brief Given the 3D global coordinate, this function loads the
   * corresponding map node in the cache thread safely and return ground
   *altitude, if necessary.
   **/
  float GetGroundAltitudeSafe(const Eigen::Vector3d& coordinate, int zone_id,
                              unsigned int resolution_id,
                              unsigned int level = 0);
  /**@brief Get the number of samples in the map cell thread safely. */
  unsigned int GetCountSafe(const Eigen::Vector3d& coordinate, int zone_id,
                            unsigned int resolution_id, unsigned int level = 0);
  /**@brief Get the number of ground samples in the map cell thread safely. */
  unsigned int GetGroundCountSafe(const Eigen::Vector3d& coordinate,
                                  int zone_id, unsigned int resolution_id,
                                  unsigned int level = 0);
};

}  // namespace pyramid_map
}  // namespace msf
}  // namespace localization
}  // namespace apollo
