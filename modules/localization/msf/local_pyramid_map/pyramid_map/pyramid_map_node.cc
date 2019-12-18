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
#include "modules/localization/msf/local_pyramid_map/pyramid_map/pyramid_map_node.h"

#include <memory>
#include <vector>
#include "modules/localization/msf/local_pyramid_map/pyramid_map/pyramid_map_matrix.h"
#include "modules/localization/msf/local_pyramid_map/pyramid_map/pyramid_map_matrix_handler.h"
#include "modules/localization/msf/local_pyramid_map/pyramid_map/pyramid_map_node_config.h"

namespace apollo {
namespace localization {
namespace msf {
namespace pyramid_map {

PyramidMapNode::PyramidMapNode() {}

PyramidMapNode::~PyramidMapNode() {}

void PyramidMapNode::Init(const BaseMapConfig* map_config) {
  map_config_ = map_config;

  map_node_config_.reset(new PyramidMapNodeConfig());
  map_node_config_->map_version_ = map_config_->GetMapVersion();
  if (map_node_config_->map_version_ == MapVersion::LOSSY_FULL_ALT_MAP ||
      map_node_config_->map_version_ == MapVersion::LOSSLESS_MAP) {
    map_node_config_->has_map_version_ = false;
    map_node_config_->has_body_md5_ = false;
  } else {
    map_node_config_->has_map_version_ = true;
    map_node_config_->has_body_md5_ = true;
  }

  is_reserved_ = false;
  data_is_ready_ = false;
  is_changed_ = false;

  map_matrix_.reset(new PyramidMapMatrix());
  map_matrix_handler_.reset(
      PyramidMapMatrixHandlerSelector::AllocPyramidMapMatrixHandler(
          map_node_config_->map_version_));
  compression_strategy_.reset(new ZlibStrategy());

  const PyramidMapConfig* pm_map_config =
      dynamic_cast<const PyramidMapConfig*>(map_config_);
  resolutions_mr_.resize(pm_map_config->resolution_num_, 0);
  for (unsigned int i = 0; i < resolutions_mr_.size(); ++i) {
    resolutions_mr_[i] =
        static_cast<float>(pm_map_config->map_resolutions_[0] *
                           std::pow(pm_map_config->resolution_ratio_, i));
  }

  InitMapMatrix(map_config_);
}

void PyramidMapNode::Init(const BaseMapConfig* map_config,
                          const MapNodeIndex& index, bool create_map_cells) {
  map_config_ = map_config;

  map_node_config_.reset(new PyramidMapNodeConfig());
  map_node_config_->node_index_ = index;
  map_node_config_->map_version_ = map_config_->GetMapVersion();
  if (map_node_config_->map_version_ == MapVersion::LOSSY_FULL_ALT_MAP ||
      map_node_config_->map_version_ == MapVersion::LOSSLESS_MAP) {
    map_node_config_->has_map_version_ = false;
    map_node_config_->has_body_md5_ = false;
  } else {
    map_node_config_->has_map_version_ = true;
    map_node_config_->has_body_md5_ = true;
  }

  left_top_corner_ =
      ComputeLeftTopCorner(*map_config_, map_node_config_->node_index_);
  is_reserved_ = false;
  data_is_ready_ = false;
  is_changed_ = false;

  map_matrix_.reset(new PyramidMapMatrix());
  map_matrix_handler_.reset(
      PyramidMapMatrixHandlerSelector::AllocPyramidMapMatrixHandler(
          map_node_config_->map_version_));
  compression_strategy_.reset(new ZlibStrategy());

  if (create_map_cells) {
    InitMapMatrix(map_config_);
  }
}

void PyramidMapNode::BottomUpBase() {
  std::shared_ptr<PyramidMapMatrix> map_matrix =
      std::dynamic_pointer_cast<PyramidMapMatrix>(map_matrix_);
  map_matrix->BottomUpBase();
}

void PyramidMapNode::BottomUpSafe() {
  std::shared_ptr<PyramidMapMatrix> map_matrix =
      std::dynamic_pointer_cast<PyramidMapMatrix>(map_matrix_);
  map_matrix->BottomUpSafe();
}

bool PyramidMapNode::AddValueIfInBound(const Eigen::Vector3d& coordinate,
                                       unsigned char intensity,
                                       unsigned int level) {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  //    Eigen::Map<Eigen::Vector2d>(coordinate.data(), 2);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = GetCoordinate(coord2d, level, &x, &y);
  if (is_success) {
    std::shared_ptr<PyramidMapMatrix> map_matrix =
        std::dynamic_pointer_cast<PyramidMapMatrix>(map_matrix_);
    map_matrix->AddSampleBase(static_cast<float>(intensity),
                              static_cast<float>(coordinate[2]), y, x, level);
    return true;
  } else {
    return false;
  }
}

void PyramidMapNode::AddValueIfInBound(
    const std::vector<Eigen::Vector3d>& coordinates,
    const std::vector<unsigned char>& intensity, unsigned int level) {
  if (coordinates.size() != intensity.size()) {
    return;
  }

  for (unsigned int i = 0; i < coordinates.size(); i++) {
    AddValueIfInBound(coordinates[i], intensity[i], level);
  }
}

bool PyramidMapNode::GetCoordinate(const Eigen::Vector2d& coordinate,
                                   unsigned int level, unsigned int* x,
                                   unsigned int* y) const {
  std::shared_ptr<PyramidMapMatrix> map_matrix =
      std::dynamic_pointer_cast<PyramidMapMatrix>(map_matrix_);

  const float& current_resolution = resolutions_mr_[level];

  unsigned int off_x = static_cast<unsigned int>(
      (coordinate[0] - left_top_corner_[0]) / current_resolution);
  unsigned int off_y = static_cast<unsigned int>(
      (coordinate[1] - left_top_corner_[1]) / current_resolution);
  if (off_x < map_matrix->GetCols(level) &&
      off_y < map_matrix->GetRows(level)) {
    *x = off_x;
    *y = off_y;
    return true;
  }
  return false;
}

Eigen::Vector2d PyramidMapNode::GetCoordinate(unsigned int level,
                                              unsigned int x,
                                              unsigned int y) const {
  const PyramidMapConfig* map_config =
      dynamic_cast<const PyramidMapConfig*>(map_config_);

  float current_resolution =
      static_cast<float>(map_config->map_resolutions_[0] *
                         std::pow(map_config->resolution_ratio_, level));

  Eigen::Vector2d coord;
  coord[0] = left_top_corner_[0] + current_resolution * static_cast<float>(x);
  coord[1] = left_top_corner_[1] + current_resolution * static_cast<float>(y);
  return coord;
}

bool PyramidMapNode::GetCoordinate(const Eigen::Vector2d& coordinate,
                                   unsigned int* x, unsigned int* y) const {
  return BaseMapNode::GetCoordinate(coordinate, x, y);
}
bool PyramidMapNode::GetCoordinate(const Eigen::Vector3d& coordinate,
                                   unsigned int* x, unsigned int* y) const {
  return BaseMapNode::GetCoordinate(coordinate, x, y);
}

Eigen::Vector2d PyramidMapNode::GetCoordinate(unsigned int x,
                                              unsigned int y) const {
  return BaseMapNode::GetCoordinate(x, y);
}

float PyramidMapNode::GetIntensitySafe(const Eigen::Vector3d& coordinate,
                                       unsigned int level) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = GetCoordinate(coord2d, level, &x, &y);

  float intensity = 0.0;
  if (is_success) {
    std::shared_ptr<PyramidMapMatrix> map_matrix =
        std::dynamic_pointer_cast<PyramidMapMatrix>(map_matrix_);
    const float* intensity_ptr = map_matrix->GetIntensitySafe(y, x, level);

    if (intensity_ptr != nullptr) {
      intensity = *intensity_ptr;
    }
  }

  return intensity;
}

float PyramidMapNode::GetIntensityVarSafe(const Eigen::Vector3d& coordinate,
                                          unsigned int level) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = GetCoordinate(coord2d, level, &x, &y);

  float intensity_var = 0.0;
  if (is_success) {
    std::shared_ptr<PyramidMapMatrix> map_matrix =
        std::dynamic_pointer_cast<PyramidMapMatrix>(map_matrix_);
    const float* intensity_var_ptr =
        map_matrix->GetIntensityVarSafe(y, x, level);

    if (intensity_var_ptr != nullptr) {
      intensity_var = *intensity_var_ptr;
    }
  }

  return intensity_var;
}

float PyramidMapNode::GetAltitudeSafe(const Eigen::Vector3d& coordinate,
                                      unsigned int level) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = GetCoordinate(coord2d, level, &x, &y);

  float altitude = 0.0;
  if (is_success) {
    std::shared_ptr<PyramidMapMatrix> map_matrix =
        std::dynamic_pointer_cast<PyramidMapMatrix>(map_matrix_);
    const float* altitude_ptr = map_matrix->GetAltitudeSafe(y, x, level);

    if (altitude_ptr != nullptr) {
      altitude = *altitude_ptr;
    }
  }

  return altitude;
}

float PyramidMapNode::GetAltitudeVarSafe(const Eigen::Vector3d& coordinate,
                                         unsigned int level) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = GetCoordinate(coord2d, level, &x, &y);

  float altitude_var = 0.0;
  if (is_success) {
    std::shared_ptr<PyramidMapMatrix> map_matrix =
        std::dynamic_pointer_cast<PyramidMapMatrix>(map_matrix_);
    const float* altitude_var_ptr = map_matrix->GetAltitudeVarSafe(y, x, level);

    if (altitude_var_ptr != nullptr) {
      altitude_var = *altitude_var_ptr;
    }
  }

  return altitude_var;
}

float PyramidMapNode::GetGroundAltitudeSafe(const Eigen::Vector3d& coordinate,
                                            unsigned int level) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = GetCoordinate(coord2d, level, &x, &y);

  float ground_altitude = 0.0;
  if (is_success) {
    std::shared_ptr<PyramidMapMatrix> map_matrix =
        std::dynamic_pointer_cast<PyramidMapMatrix>(map_matrix_);
    const float* ground_altitude_ptr =
        map_matrix->GetGroundAltitudeSafe(y, x, level);

    if (ground_altitude_ptr != nullptr) {
      ground_altitude = *ground_altitude_ptr;
    }
  }

  return ground_altitude;
}

unsigned int PyramidMapNode::GetCountSafe(const Eigen::Vector3d& coordinate,
                                          unsigned int level) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = GetCoordinate(coord2d, level, &x, &y);

  unsigned int count = 0;
  if (is_success) {
    std::shared_ptr<PyramidMapMatrix> map_matrix =
        std::dynamic_pointer_cast<PyramidMapMatrix>(map_matrix_);
    const unsigned int* count_ptr = map_matrix->GetCountSafe(y, x, level);

    if (count_ptr != nullptr) {
      count = *count_ptr;
    }
  }

  return count;
}

unsigned int PyramidMapNode::GetGroundCountSafe(
    const Eigen::Vector3d& coordinate, unsigned int level) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = GetCoordinate(coord2d, level, &x, &y);

  unsigned int ground_count = 0;
  if (is_success) {
    std::shared_ptr<PyramidMapMatrix> map_matrix =
        std::dynamic_pointer_cast<PyramidMapMatrix>(map_matrix_);
    const unsigned int* ground_count_ptr =
        map_matrix->GetGroundCountSafe(y, x, level);

    if (ground_count_ptr != nullptr) {
      ground_count = *ground_count_ptr;
    }
  }

  return ground_count;
}

float PyramidMapNode::GetIntensity(const Eigen::Vector3d& coordinate,
                                   unsigned int level) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = GetCoordinate(coord2d, level, &x, &y);

  float intensity = 0.0;
  if (is_success) {
    std::shared_ptr<PyramidMapMatrix> map_matrix =
        std::dynamic_pointer_cast<PyramidMapMatrix>(map_matrix_);
    const float* intensity_ptr = map_matrix->GetIntensity(y, x, level);

    if (intensity_ptr != nullptr) {
      intensity = *intensity_ptr;
    }
  }

  return intensity;
}

float PyramidMapNode::GetIntensityVar(const Eigen::Vector3d& coordinate,
                                      unsigned int level) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = GetCoordinate(coord2d, level, &x, &y);

  float intensity_var = 0.0;
  if (is_success) {
    std::shared_ptr<PyramidMapMatrix> map_matrix =
        std::dynamic_pointer_cast<PyramidMapMatrix>(map_matrix_);
    const float* intensity_var_ptr = map_matrix->GetIntensityVar(y, x, level);

    if (intensity_var_ptr != nullptr) {
      intensity_var = *intensity_var_ptr;
    }
  }

  return intensity_var;
}

float PyramidMapNode::GetAltitude(const Eigen::Vector3d& coordinate,
                                  unsigned int level) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = GetCoordinate(coord2d, level, &x, &y);

  float altitude = 0.0;
  if (is_success) {
    std::shared_ptr<PyramidMapMatrix> map_matrix =
        std::dynamic_pointer_cast<PyramidMapMatrix>(map_matrix_);
    const float* altitude_ptr = map_matrix->GetAltitude(y, x, level);

    if (altitude_ptr != nullptr) {
      altitude = *altitude_ptr;
    }
  }

  return altitude;
}

float PyramidMapNode::GetAltitudeVar(const Eigen::Vector3d& coordinate,
                                     unsigned int level) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = GetCoordinate(coord2d, level, &x, &y);

  float altitude_var = 0.0;
  if (is_success) {
    std::shared_ptr<PyramidMapMatrix> map_matrix =
        std::dynamic_pointer_cast<PyramidMapMatrix>(map_matrix_);
    const float* altitude_var_ptr = map_matrix->GetAltitudeVar(y, x, level);

    if (altitude_var_ptr != nullptr) {
      altitude_var = *altitude_var_ptr;
    }
  }

  return altitude_var;
}

float PyramidMapNode::GetGroundAltitude(const Eigen::Vector3d& coordinate,
                                        unsigned int level) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = GetCoordinate(coord2d, level, &x, &y);

  float ground_altitude = 0.0;
  if (is_success) {
    std::shared_ptr<PyramidMapMatrix> map_matrix =
        std::dynamic_pointer_cast<PyramidMapMatrix>(map_matrix_);
    const float* ground_altitude_ptr =
        map_matrix->GetGroundAltitude(y, x, level);

    if (ground_altitude_ptr != nullptr) {
      ground_altitude = *ground_altitude_ptr;
    }
  }

  return ground_altitude;
}

unsigned int PyramidMapNode::GetCount(const Eigen::Vector3d& coordinate,
                                      unsigned int level) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = GetCoordinate(coord2d, level, &x, &y);

  unsigned int count = 0;
  if (is_success) {
    std::shared_ptr<PyramidMapMatrix> map_matrix =
        std::dynamic_pointer_cast<PyramidMapMatrix>(map_matrix_);
    const unsigned int* count_ptr = map_matrix->GetCount(y, x, level);

    if (count_ptr != nullptr) {
      count = *count_ptr;
    }
  }

  return count;
}

unsigned int PyramidMapNode::GetGroundCount(const Eigen::Vector3d& coordinate,
                                            unsigned int level) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = GetCoordinate(coord2d, level, &x, &y);

  unsigned int ground_count = 0;
  if (is_success) {
    std::shared_ptr<PyramidMapMatrix> map_matrix =
        std::dynamic_pointer_cast<PyramidMapMatrix>(map_matrix_);
    const unsigned int* ground_count_ptr =
        map_matrix->GetGroundCount(y, x, level);

    if (ground_count_ptr != nullptr) {
      ground_count = *ground_count_ptr;
    }
  }

  return ground_count;
}

double PyramidMapNode::ComputeMeanIntensity(unsigned int level) {
  std::shared_ptr<PyramidMapMatrix> map_matrix =
      std::dynamic_pointer_cast<PyramidMapMatrix>(map_matrix_);
  return map_matrix->ComputeMeanIntensity(level);
}

}  // namespace pyramid_map
}  // namespace msf
}  // namespace localization
}  // namespace apollo
