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

#include "modules/localization/msf/local_pyramid_map/pyramid_map/pyramid_map.h"

#include "modules/localization/msf/local_pyramid_map/pyramid_map/pyramid_map_node.h"

namespace apollo {
namespace localization {
namespace msf {
namespace pyramid_map {

PyramidMap::PyramidMap(PyramidMapConfig* config) : BaseMap(config) {}

PyramidMap::~PyramidMap() {}

float PyramidMap::GetIntensitySafe(const Eigen::Vector3d& coordinate,
                                   int zone_id, unsigned int resolution_id,
                                   unsigned int level) {
  MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(GetMapConfig(), coordinate,
                                                     resolution_id, zone_id);
  PyramidMapNode* node = dynamic_cast<PyramidMapNode*>(GetMapNodeSafe(index));
  return node->GetIntensitySafe(coordinate, level);
}

float PyramidMap::GetIntensityVarSafe(const Eigen::Vector3d& coordinate,
                                      int zone_id, unsigned int resolution_id,
                                      unsigned int level) {
  MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(GetMapConfig(), coordinate,
                                                     resolution_id, zone_id);
  PyramidMapNode* node = dynamic_cast<PyramidMapNode*>(GetMapNodeSafe(index));
  return node->GetIntensityVarSafe(coordinate, level);
}

float PyramidMap::GetAltitudeSafe(const Eigen::Vector3d& coordinate,
                                  int zone_id, unsigned int resolution_id,
                                  unsigned int level) {
  MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(GetMapConfig(), coordinate,
                                                     resolution_id, zone_id);
  PyramidMapNode* node = dynamic_cast<PyramidMapNode*>(GetMapNodeSafe(index));
  return node->GetAltitudeSafe(coordinate, level);
}

float PyramidMap::GetAltitudeVarSafe(const Eigen::Vector3d& coordinate,
                                     int zone_id, unsigned int resolution_id,
                                     unsigned int level) {
  MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(GetMapConfig(), coordinate,
                                                     resolution_id, zone_id);
  PyramidMapNode* node = dynamic_cast<PyramidMapNode*>(GetMapNodeSafe(index));
  return node->GetAltitudeVarSafe(coordinate, level);
}

float PyramidMap::GetGroundAltitudeSafe(const Eigen::Vector3d& coordinate,
                                        int zone_id, unsigned int resolution_id,
                                        unsigned int level) {
  MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(GetMapConfig(), coordinate,
                                                     resolution_id, zone_id);
  PyramidMapNode* node = dynamic_cast<PyramidMapNode*>(GetMapNodeSafe(index));
  return node->GetGroundAltitudeSafe(coordinate, level);
}

unsigned int PyramidMap::GetCountSafe(const Eigen::Vector3d& coordinate,
                                      int zone_id, unsigned int resolution_id,
                                      unsigned int level) {
  MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(GetMapConfig(), coordinate,
                                                     resolution_id, zone_id);
  PyramidMapNode* node = dynamic_cast<PyramidMapNode*>(GetMapNodeSafe(index));
  return node->GetCountSafe(coordinate, level);
}

unsigned int PyramidMap::GetGroundCountSafe(const Eigen::Vector3d& coordinate,
                                            int zone_id,
                                            unsigned int resolution_id,
                                            unsigned int level) {
  MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(GetMapConfig(), coordinate,
                                                     resolution_id, zone_id);
  PyramidMapNode* node = dynamic_cast<PyramidMapNode*>(GetMapNodeSafe(index));
  return node->GetGroundCountSafe(coordinate, level);
}

}  // namespace pyramid_map
}  // namespace msf
}  // namespace localization
}  // namespace apollo
