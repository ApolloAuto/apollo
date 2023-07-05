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

#include "modules/localization/msf/local_map/lossless_map/lossless_map_node.h"

#include "cyber/common/log.h"
#include "modules/localization/msf/local_map/lossless_map/lossless_map_config.h"

namespace apollo {
namespace localization {
namespace msf {
LosslessMapNode::LosslessMapNode()
    : BaseMapNode(new LosslessMapMatrix(), new ZlibStrategy()) {}

LosslessMapNode::~LosslessMapNode() {}

void LosslessMapNode::SetValue(const Eigen::Vector3d& coordinate,
                               unsigned char intensity) {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = GetCoordinate(coord2d, &x, &y);
  DCHECK(is_success);
  LosslessMapCell& map_cell =
      static_cast<LosslessMapMatrix*>(map_matrix_)->GetMapCell(y, x);
  map_cell.SetValue(coordinate[2], intensity);
  if (min_altitude_ > coordinate[2]) {
    min_altitude_ = static_cast<float>(coordinate[2]);
  }
  is_changed_ = true;
}

bool LosslessMapNode::SetValueIfInBound(const Eigen::Vector3d& coordinate,
                                        unsigned char intensity) {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = GetCoordinate(coord2d, &x, &y);
  if (is_success) {
    LosslessMapCell& map_cell =
        static_cast<LosslessMapMatrix*>(map_matrix_)->GetMapCell(y, x);
    map_cell.SetValue(coordinate[2], intensity);
    if (min_altitude_ > coordinate[2]) {
      min_altitude_ = static_cast<float>(coordinate[2]);
    }
    is_changed_ = true;
    return true;
  } else {
    return false;
  }
}

void LosslessMapNode::SetValueLayer(const Eigen::Vector3d& coordinate,
                                    unsigned char intensity) {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = GetCoordinate(coord2d, &x, &y);
  DCHECK(is_success);
  LosslessMapCell& map_cell =
      static_cast<LosslessMapMatrix*>(map_matrix_)->GetMapCell(y, x);
  map_cell.SetValueLayer(
      coordinate[2], intensity,
      static_cast<const LosslessMapConfig*>(map_config_)->map_layer_alt_thres_);
  is_changed_ = true;
}

void LosslessMapNode::GetValue(const Eigen::Vector3d& coordinate,
                               std::vector<unsigned char>* values) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = GetCoordinate(coord2d, &x, &y);
  DCHECK(is_success);
  LosslessMapCell& map_cell =
      static_cast<LosslessMapMatrix*>(map_matrix_)->GetMapCell(y, x);
  map_cell.GetValue(values);
}

void LosslessMapNode::GetVar(const Eigen::Vector3d& coordinate,
                             std::vector<float>* vars) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = GetCoordinate(coord2d, &x, &y);
  DCHECK(is_success);
  LosslessMapCell& map_cell =
      static_cast<LosslessMapMatrix*>(map_matrix_)->GetMapCell(y, x);
  map_cell.GetVar(vars);
}

void LosslessMapNode::GetAlt(const Eigen::Vector3d& coordinate,
                             std::vector<float>* alts) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = GetCoordinate(coord2d, &x, &y);
  DCHECK(is_success);
  LosslessMapCell& map_cell =
      static_cast<LosslessMapMatrix*>(map_matrix_)->GetMapCell(y, x);
  map_cell.GetAlt(alts);
}

void LosslessMapNode::GetAltVar(const Eigen::Vector3d& coordinate,
                                std::vector<float>* alt_vars) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = GetCoordinate(coord2d, &x, &y);
  DCHECK(is_success);
  LosslessMapCell& map_cell =
      static_cast<LosslessMapMatrix*>(map_matrix_)->GetMapCell(y, x);
  map_cell.GetAltVar(alt_vars);
}

void LosslessMapNode::GetCount(const Eigen::Vector3d& coordinate,
                               std::vector<unsigned int>* counts) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = GetCoordinate(coord2d, &x, &y);
  DCHECK(is_success);
  LosslessMapCell& map_cell =
      static_cast<LosslessMapMatrix*>(map_matrix_)->GetMapCell(y, x);
  map_cell.GetCount(counts);
}

unsigned char LosslessMapNode::GetValue(
    const Eigen::Vector3d& coordinate) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = GetCoordinate(coord2d, &x, &y);
  DCHECK(is_success);
  LosslessMapCell& map_cell =
      static_cast<LosslessMapMatrix*>(map_matrix_)->GetMapCell(y, x);
  return map_cell.GetValue();
}

float LosslessMapNode::GetVar(const Eigen::Vector3d& coordinate) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = GetCoordinate(coord2d, &x, &y);
  DCHECK(is_success);
  LosslessMapCell& map_cell =
      static_cast<LosslessMapMatrix*>(map_matrix_)->GetMapCell(y, x);
  return map_cell.GetVar();
}

float LosslessMapNode::GetAlt(const Eigen::Vector3d& coordinate) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = GetCoordinate(coord2d, &x, &y);
  DCHECK(is_success);
  LosslessMapCell& map_cell =
      static_cast<LosslessMapMatrix*>(map_matrix_)->GetMapCell(y, x);
  return map_cell.GetAlt();
}

float LosslessMapNode::GetAltVar(const Eigen::Vector3d& coordinate) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = GetCoordinate(coord2d, &x, &y);
  DCHECK(is_success);
  LosslessMapCell& map_cell =
      static_cast<LosslessMapMatrix*>(map_matrix_)->GetMapCell(y, x);
  return map_cell.GetAltVar();
}

unsigned int LosslessMapNode::GetCount(
    const Eigen::Vector3d& coordinate) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = GetCoordinate(coord2d, &x, &y);
  DCHECK(is_success);
  LosslessMapCell& map_cell =
      static_cast<LosslessMapMatrix*>(map_matrix_)->GetMapCell(y, x);
  return map_cell.GetCount();
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
