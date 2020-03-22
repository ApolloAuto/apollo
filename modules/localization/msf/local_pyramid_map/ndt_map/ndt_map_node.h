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

#include "Eigen/Core"

#include "modules/localization/msf/common/util/file_utility.h"
#include "modules/localization/msf/local_pyramid_map/base_map/base_map_node.h"
#include "modules/localization/msf/local_pyramid_map/ndt_map/ndt_map_config.h"
#include "modules/localization/msf/local_pyramid_map/ndt_map/ndt_map_matrix.h"

namespace apollo {
namespace localization {
namespace msf {
namespace pyramid_map {

class NdtMapNode : public BaseMapNode {
 public:
  NdtMapNode();
  ~NdtMapNode();

  void Init(const BaseMapConfig* map_config);
  void Init(const BaseMapConfig* map_config, const MapNodeIndex& index,
            bool create_map_cells = true);

  /**@brief Get the resolution of this map nodex. */
  inline float GetMapResolutionZ() const {
    return static_cast<const NdtMapConfig*>(map_config_)
        ->map_resolutions_z_[index_.resolution_id_];
  }

  /**@brief Given the local x, y, altitude index,
   * return the global coordinate.
   */
  Eigen::Vector3d GetCoordinate3D(unsigned int x, unsigned int y,
                                  int altitude_index) const;

  /**@brief Given the local x, y, altitude index,
   * return the global coordinate.
   */
  Eigen::Vector3d GetCoordinateCenter3D(unsigned int x, unsigned int y,
                                        int altitude_index) const;

  /**@brief Combine two map nodes (Reduce operation in mapreduce).
   * The result is saved in map_node. */
  static void Reduce(NdtMapNode* map_node, const NdtMapNode& map_node_new);

  /**@brief The number of cells with elements.*/
  unsigned int num_valid_cells_;
  /**@brief The number of single cells with elements. */
  unsigned int num_valid_single_cells_;
};

}  // namespace pyramid_map
}  // namespace msf
}  // namespace localization
}  // namespace apollo
