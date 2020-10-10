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

#include "modules/localization/msf/local_pyramid_map/ndt_map/ndt_map_node.h"

#include "modules/localization/msf/local_pyramid_map/ndt_map/ndt_map_matrix_handler.h"
#include "modules/localization/msf/local_pyramid_map/ndt_map/ndt_map_node_config.h"

namespace apollo {
namespace localization {
namespace msf {
namespace pyramid_map {

NdtMapNode::NdtMapNode() {}
NdtMapNode::~NdtMapNode() {}

void NdtMapNode::Init(const BaseMapConfig* map_config) {
  map_config_ = map_config;

  map_node_config_.reset(new NdtMapNodeConfig());
  map_node_config_->map_version_ = map_config_->GetMapVersion();
  map_node_config_->has_map_version_ = false;
  map_node_config_->has_body_md5_ = false;
  is_reserved_ = false;
  data_is_ready_ = false;
  is_changed_ = false;
  num_valid_cells_ = 0;
  num_valid_single_cells_ = 0;

  map_matrix_.reset(new NdtMapMatrix());
  map_matrix_handler_.reset(
      NdtMapMatrixHandlerSelector::AllocNdtMapMatrixHandler());
  compression_strategy_.reset(new ZlibStrategy());
  InitMapMatrix(map_config_);
}
void NdtMapNode::Init(const BaseMapConfig* map_config,
                      const MapNodeIndex& index, bool create_map_cells) {
  map_config_ = map_config;

  map_node_config_.reset(new NdtMapNodeConfig());
  map_node_config_->node_index_ = index;
  map_node_config_->map_version_ = map_config_->GetMapVersion();
  left_top_corner_ =
      ComputeLeftTopCorner(*map_config_, map_node_config_->node_index_);
  map_node_config_->has_map_version_ = false;
  map_node_config_->has_body_md5_ = false;
  is_reserved_ = false;
  data_is_ready_ = false;
  is_changed_ = false;
  num_valid_cells_ = 0;
  num_valid_single_cells_ = 0;

  map_matrix_.reset(new NdtMapMatrix());
  map_matrix_handler_.reset(
      NdtMapMatrixHandlerSelector::AllocNdtMapMatrixHandler());
  compression_strategy_.reset(new ZlibStrategy());
  if (create_map_cells) {
    InitMapMatrix(map_config_);
  }
}

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
      static_cast<NdtMapMatrix*>(map_node->map_matrix_.get()),
      static_cast<const NdtMapMatrix&>(*map_node_new.map_matrix_));
}

}  // namespace pyramid_map
}  // namespace msf
}  // namespace localization
}  // namespace apollo
