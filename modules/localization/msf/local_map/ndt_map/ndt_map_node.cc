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

#include "modules/localization/msf/local_map/ndt_map/ndt_map_node.h"
#include "modules/localization/msf/local_map/ndt_map/ndt_map_matrix.h"

namespace apollo {
namespace localization {
namespace msf {

NdtMapNode::NdtMapNode() : BaseMapNode(new NdtMapMatrix(), new ZlibStrategy()) {
  num_valid_cells_ = 0;
  num_valid_single_cells_ = 0;
}
NdtMapNode::~NdtMapNode() {}
Eigen::Vector3d NdtMapNode::GetCoordinate3D(unsigned int x, unsigned int y,
                                            int altitude_index) const {
  const Eigen::Vector2d& left_top_corner = GetLeftTopCorner();
  Eigen::Vector2d coord_2d;
  coord_2d[0] =
      left_top_corner[0] + (static_cast<double>(x)) * GetMapResolution();
  coord_2d[1] =
      left_top_corner[1] + (static_cast<double>(y)) * GetMapResolution();

  double altitude =
      NdtMapCells::CalAltitude(GetMapResolutionZ(), altitude_index);
  Eigen::Vector3d coord_3d;
  coord_3d[0] = coord_2d[0];
  coord_3d[1] = coord_2d[1];
  coord_3d[2] = altitude;

  return coord_3d;
}

Eigen::Vector3d NdtMapNode::GetCoordinateCenter3D(unsigned int x,
                                                  unsigned int y,
                                                  int altitude_index) const {
  const Eigen::Vector2d& left_top_corner = GetLeftTopCorner();
  Eigen::Vector2d coord_2d;
  coord_2d[0] =
      left_top_corner[0] + (static_cast<double>(x) + 0.5) * GetMapResolution();
  coord_2d[1] =
      left_top_corner[1] + (static_cast<double>(y) + 0.5) * GetMapResolution();

  double altitude =
      NdtMapCells::CalAltitude(GetMapResolutionZ(), altitude_index);
  Eigen::Vector3d coord_3d;
  coord_3d[0] = coord_2d[0];
  coord_3d[1] = coord_2d[1];
  coord_3d[2] = altitude;

  return coord_3d;
}

void NdtMapNode::Reduce(NdtMapNode* map_node, const NdtMapNode& map_node_new) {
  assert(map_node->index_.m_ == map_node_new.index_.m_);
  assert(map_node->index_.n_ == map_node_new.index_.n_);
  assert(map_node->index_.resolution_id_ == map_node_new.index_.resolution_id_);
  assert(map_node->index_.zone_id_ == map_node_new.index_.zone_id_);
  NdtMapMatrix::Reduce(
      static_cast<NdtMapMatrix*>(map_node->map_matrix_),
      static_cast<const NdtMapMatrix&>(*map_node_new.map_matrix_));
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
