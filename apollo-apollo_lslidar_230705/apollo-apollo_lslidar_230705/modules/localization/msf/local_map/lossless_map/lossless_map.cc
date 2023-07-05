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

#include "modules/localization/msf/local_map/lossless_map/lossless_map.h"

#include "cyber/common/log.h"
#include "modules/localization/msf/local_map/lossless_map/lossless_map_node.h"

namespace apollo {
namespace localization {
namespace msf {
LosslessMap::LosslessMap(LosslessMapConfig* config) : BaseMap(config) {}

LosslessMap::~LosslessMap() {}

void LosslessMap::SetValue(const Eigen::Vector3d& coordinate, int zone_id,
                           unsigned char intensity) {
  for (size_t i = 0; i < map_config_->map_resolutions_.size(); ++i) {
    MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(
        *map_config_, coordinate, static_cast<unsigned int>(i), zone_id);
    LosslessMapNode* node =
        static_cast<LosslessMapNode*>(GetMapNodeSafe(index));
    node->SetValue(coordinate, intensity);
  }
}

void LosslessMap::SetValueLayer(const Eigen::Vector3d& coordinate, int zone_id,
                                unsigned char intensity) {
  for (size_t i = 0; i < map_config_->map_resolutions_.size(); ++i) {
    MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(
        *map_config_, coordinate, static_cast<unsigned int>(i), zone_id);
    LosslessMapNode* node =
        static_cast<LosslessMapNode*>(GetMapNodeSafe(index));
    node->SetValueLayer(coordinate, intensity);
  }
}

void LosslessMap::GetValue(const Eigen::Vector3d& coordinate, int zone_id,
                           unsigned int resolution_id,
                           std::vector<unsigned char>* values) {
  DCHECK_LT(resolution_id, map_config_->map_resolutions_.size());
  MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(*map_config_, coordinate,
                                                     resolution_id, zone_id);
  LosslessMapNode* node = static_cast<LosslessMapNode*>(GetMapNode(index));
  node->GetValue(coordinate, values);
}

void LosslessMap::GetValueSafe(const Eigen::Vector3d& coordinate, int zone_id,
                               unsigned int resolution_id,
                               std::vector<unsigned char>* values) {
  DCHECK_LT(resolution_id, map_config_->map_resolutions_.size());
  MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(*map_config_, coordinate,
                                                     resolution_id, zone_id);
  LosslessMapNode* node = static_cast<LosslessMapNode*>(GetMapNodeSafe(index));
  node->GetValue(coordinate, values);
}

void LosslessMap::GetVar(const Eigen::Vector3d& coordinate, int zone_id,
                         unsigned int resolution_id, std::vector<float>* vars) {
  DCHECK_LT(resolution_id, map_config_->map_resolutions_.size());
  MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(*map_config_, coordinate,
                                                     resolution_id, zone_id);
  LosslessMapNode* node = static_cast<LosslessMapNode*>(GetMapNode(index));
  node->GetVar(coordinate, vars);
}

void LosslessMap::GetVarSafe(const Eigen::Vector3d& coordinate, int zone_id,
                             unsigned int resolution_id,
                             std::vector<float>* vars) {
  DCHECK_LT(resolution_id, map_config_->map_resolutions_.size());
  MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(*map_config_, coordinate,
                                                     resolution_id, zone_id);
  LosslessMapNode* node = static_cast<LosslessMapNode*>(GetMapNodeSafe(index));
  node->GetVar(coordinate, vars);
}

void LosslessMap::GetAlt(const Eigen::Vector3d& coordinate, int zone_id,
                         unsigned int resolution_id, std::vector<float>* alts) {
  DCHECK_LT(resolution_id, map_config_->map_resolutions_.size());
  MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(*map_config_, coordinate,
                                                     resolution_id, zone_id);
  LosslessMapNode* node = static_cast<LosslessMapNode*>(GetMapNode(index));
  node->GetAlt(coordinate, alts);
}

void LosslessMap::GetAltSafe(const Eigen::Vector3d& coordinate, int zone_id,
                             unsigned int resolution_id,
                             std::vector<float>* alts) {
  DCHECK_LT(resolution_id, map_config_->map_resolutions_.size());
  MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(*map_config_, coordinate,
                                                     resolution_id, zone_id);
  LosslessMapNode* node = static_cast<LosslessMapNode*>(GetMapNodeSafe(index));
  node->GetAlt(coordinate, alts);
}

void LosslessMap::GetAltVar(const Eigen::Vector3d& coordinate, int zone_id,
                            unsigned int resolution_id,
                            std::vector<float>* alt_vars) {
  DCHECK_LT(resolution_id, map_config_->map_resolutions_.size());
  MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(*map_config_, coordinate,
                                                     resolution_id, zone_id);
  LosslessMapNode* node = static_cast<LosslessMapNode*>(GetMapNode(index));
  node->GetAltVar(coordinate, alt_vars);
}

void LosslessMap::GetAltVarSafe(const Eigen::Vector3d& coordinate, int zone_id,
                                unsigned int resolution_id,
                                std::vector<float>* alt_vars) {
  DCHECK_LT(resolution_id, map_config_->map_resolutions_.size());
  MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(*map_config_, coordinate,
                                                     resolution_id, zone_id);
  LosslessMapNode* node = static_cast<LosslessMapNode*>(GetMapNodeSafe(index));
  node->GetAltVar(coordinate, alt_vars);
}

void LosslessMap::GetCount(const Eigen::Vector3d& coordinate, int zone_id,
                           unsigned int resolution_id,
                           std::vector<unsigned int>* counts) {
  DCHECK_LT(resolution_id, map_config_->map_resolutions_.size());
  MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(*map_config_, coordinate,
                                                     resolution_id, zone_id);
  LosslessMapNode* node = static_cast<LosslessMapNode*>(GetMapNode(index));
  node->GetCount(coordinate, counts);
}

void LosslessMap::GetCountSafe(const Eigen::Vector3d& coordinate, int zone_id,
                               unsigned int resolution_id,
                               std::vector<unsigned int>* counts) {
  DCHECK_LT(resolution_id, map_config_->map_resolutions_.size());
  MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(*map_config_, coordinate,
                                                     resolution_id, zone_id);
  LosslessMapNode* node = static_cast<LosslessMapNode*>(GetMapNodeSafe(index));
  node->GetCount(coordinate, counts);
}

unsigned char LosslessMap::GetValue(const Eigen::Vector3d& coordinate,
                                    int zone_id, unsigned int resolution_id) {
  DCHECK_LT(resolution_id, map_config_->map_resolutions_.size());
  MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(*map_config_, coordinate,
                                                     resolution_id, zone_id);
  LosslessMapNode* node = static_cast<LosslessMapNode*>(GetMapNode(index));
  return node->GetValue(coordinate);
}

unsigned char LosslessMap::GetValueSafe(const Eigen::Vector3d& coordinate,
                                        int zone_id,
                                        unsigned int resolution_id) {
  DCHECK_LT(resolution_id, map_config_->map_resolutions_.size());
  MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(*map_config_, coordinate,
                                                     resolution_id, zone_id);
  LosslessMapNode* node = static_cast<LosslessMapNode*>(GetMapNodeSafe(index));
  return node->GetValue(coordinate);
}

float LosslessMap::GetVar(const Eigen::Vector3d& coordinate, int zone_id,
                          unsigned int resolution_id) {
  DCHECK_LT(resolution_id, map_config_->map_resolutions_.size());
  MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(*map_config_, coordinate,
                                                     resolution_id, zone_id);
  LosslessMapNode* node = static_cast<LosslessMapNode*>(GetMapNode(index));
  return node->GetVar(coordinate);
}

float LosslessMap::GetVarSafe(const Eigen::Vector3d& coordinate, int zone_id,
                              unsigned int resolution_id) {
  DCHECK_LT(resolution_id, map_config_->map_resolutions_.size());
  MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(*map_config_, coordinate,
                                                     resolution_id, zone_id);
  LosslessMapNode* node = static_cast<LosslessMapNode*>(GetMapNodeSafe(index));
  return node->GetVar(coordinate);
}

float LosslessMap::GetAlt(const Eigen::Vector3d& coordinate, int zone_id,
                          unsigned int resolution_id) {
  DCHECK_LT(resolution_id, map_config_->map_resolutions_.size());
  MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(*map_config_, coordinate,
                                                     resolution_id, zone_id);
  LosslessMapNode* node = static_cast<LosslessMapNode*>(GetMapNode(index));
  return node->GetAlt(coordinate);
}

float LosslessMap::GetAltSafe(const Eigen::Vector3d& coordinate, int zone_id,
                              unsigned int resolution_id) {
  DCHECK_LT(resolution_id, map_config_->map_resolutions_.size());
  MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(*map_config_, coordinate,
                                                     resolution_id, zone_id);
  LosslessMapNode* node = static_cast<LosslessMapNode*>(GetMapNodeSafe(index));
  return node->GetAlt(coordinate);
}

float LosslessMap::GetAltVar(const Eigen::Vector3d& coordinate, int zone_id,
                             unsigned int resolution_id) {
  DCHECK_LT(resolution_id, map_config_->map_resolutions_.size());
  MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(*map_config_, coordinate,
                                                     resolution_id, zone_id);
  LosslessMapNode* node = static_cast<LosslessMapNode*>(GetMapNode(index));
  return node->GetAltVar(coordinate);
}

float LosslessMap::GetAltVarSafe(const Eigen::Vector3d& coordinate, int zone_id,
                                 unsigned int resolution_id) {
  DCHECK_LT(resolution_id, map_config_->map_resolutions_.size());
  MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(*map_config_, coordinate,
                                                     resolution_id, zone_id);
  LosslessMapNode* node = static_cast<LosslessMapNode*>(GetMapNodeSafe(index));
  return node->GetAltVar(coordinate);
}

unsigned int LosslessMap::GetCount(const Eigen::Vector3d& coordinate,
                                   int zone_id, unsigned int resolution_id) {
  DCHECK_LT(resolution_id, map_config_->map_resolutions_.size());
  MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(*map_config_, coordinate,
                                                     resolution_id, zone_id);
  LosslessMapNode* node = static_cast<LosslessMapNode*>(GetMapNode(index));
  return node->GetCount(coordinate);
}

unsigned int LosslessMap::GetCountSafe(const Eigen::Vector3d& coordinate,
                                       int zone_id,
                                       unsigned int resolution_id) {
  DCHECK_LT(resolution_id, map_config_->map_resolutions_.size());
  MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(*map_config_, coordinate,
                                                     resolution_id, zone_id);
  LosslessMapNode* node = static_cast<LosslessMapNode*>(GetMapNodeSafe(index));
  return node->GetCount(coordinate);
}

void LosslessMap::PreloadMapArea(const Eigen::Vector3d& location,
                                 const Eigen::Vector3d& trans_diff,
                                 unsigned int resolution_id,
                                 unsigned int zone_id) {
  BaseMap::PreloadMapArea(location, trans_diff, resolution_id, zone_id);
}

bool LosslessMap::LoadMapArea(const Eigen::Vector3d& seed_pt3d,
                              unsigned int resolution_id, unsigned int zone_id,
                              int filter_size_x, int filter_size_y) {
  BaseMap::LoadMapArea(seed_pt3d, resolution_id, zone_id, filter_size_x,
                       filter_size_y);
  return true;
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
