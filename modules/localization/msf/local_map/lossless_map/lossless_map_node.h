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

#pragma once

#include <vector>

#include "modules/localization/msf/local_map/base_map/base_map_node.h"
#include "modules/localization/msf/local_map/lossless_map/lossless_map_matrix.h"

namespace apollo {
namespace localization {
namespace msf {

class LosslessMapNode : public BaseMapNode {
 public:
  LosslessMapNode();
  ~LosslessMapNode();

  /**@brief Set the value of a pixel in the map node.
   * @param <coordinate> The 3D global coordinate.
   * @param <intensity> The reflectance intensity.
   */
  void SetValue(const Eigen::Vector3d& coordinate, unsigned char intensity);
  /**@brief Set the value of a pixel in the map node if the pixel in the node.
   * @param <coordinate> The 3D global coordinate.
   * @param <intensity> The reflectance intensity.
   * @param <return> True, if pixel in the bound of the node, else False.
   * */
  bool SetValueIfInBound(const Eigen::Vector3d& coordinate,
                         unsigned char intensity);
  /**@brief Set the value of a pixel in a layer in the map node.
   * @param <coordinate> The 3D global coordinate. The z is used as the altitude
   * for the layer match.
   * @param <intensity> The reflectance intensity.
   */
  void SetValueLayer(const Eigen::Vector3d& coordinate,
                     unsigned char intensity);
  /**@brief Given the 3D global coordinate, get the map cell average intensity
   * of each layer. */
  void GetValue(const Eigen::Vector3d& coordinate,
                std::vector<unsigned char>* values) const;
  /**@brief Given the 3D global coordinate, get the map cell variance of the
   * intensity of each layer. */
  void GetVar(const Eigen::Vector3d& coordinate,
              std::vector<float>* vars) const;
  /**@brief Given the 3D global coordinate, get the map cell's average altitude
   * of each layer. */
  void GetAlt(const Eigen::Vector3d& coordinate,
              std::vector<float>* alts) const;
  /**@brief Given the 3D global coordinate, get the map cell's variance of the
   * altitude of each layer. */
  void GetAltVar(const Eigen::Vector3d& coordinate,
                 std::vector<float>* alt_vars) const;
  /**@brief Given the 3D global coordinate, get the map cell's count of the
   * samples of each layer. */
  void GetCount(const Eigen::Vector3d& coordinate,
                std::vector<unsigned int>* counts) const;
  /**@brief Given the 3D global coordinate, get the map cell average intensity.
   */
  unsigned char GetValue(const Eigen::Vector3d& coordinate) const;
  /**@brief Given the 3D global coordinate, get the map cell variance of the
   * intensity. */
  float GetVar(const Eigen::Vector3d& coordinate) const;
  /**@brief Given the 3D global coordinate, get the map cell's average altitude.
   */
  float GetAlt(const Eigen::Vector3d& coordinate) const;
  /**@brief Given the 3D global coordinate, get the map cell's variance of the
   * altitude */
  float GetAltVar(const Eigen::Vector3d& coordinate) const;
  /**@brief Given the 3D global coordinate, get the map cell's count of the
   * samples. */
  unsigned int GetCount(const Eigen::Vector3d& coordinate) const;
  /**@brief Get the map cell average intensity. */
  unsigned char GetValue(unsigned int row, unsigned int col) const {
    return static_cast<LosslessMapMatrix*>(map_matrix_)
        ->GetMapCell(row, col)
        .GetValue();
  }
  /**@brief Get the map cell variance of the intensity. */
  float GetVar(unsigned int row, unsigned int col) const {
    return static_cast<LosslessMapMatrix*>(map_matrix_)
        ->GetMapCell(row, col)
        .GetVar();
  }
  /**@brief Get the map cell's average altitude. */
  float GetAlt(unsigned int row, unsigned int col) const {
    return static_cast<LosslessMapMatrix*>(map_matrix_)
        ->GetMapCell(row, col)
        .GetAlt();
  }
  /**@brief Get the map cell's variance of the altitude */
  float GetAltVar(unsigned int row, unsigned int col) const {
    return static_cast<LosslessMapMatrix*>(map_matrix_)
        ->GetMapCell(row, col)
        .GetAltVar();
  }
  /**@brief Get the map cell's count of the samples. */
  unsigned int GetCount(unsigned int row, unsigned int col) const {
    return static_cast<LosslessMapMatrix*>(map_matrix_)
        ->GetMapCell(row, col)
        .GetCount();
  }
  /**@brief Get the constant map cell given the coordinate. */
  inline const LosslessMapSingleCell& GetFirstMapCell(unsigned int row,
                                                      unsigned int col) const {
    return static_cast<LosslessMapMatrix*>(map_matrix_)
        ->GetMapCell(row, col)
        .map_cells[0];
  }
  inline LosslessMapSingleCell& GetFirstMapCell(unsigned int row,
                                                unsigned int col) {
    return static_cast<LosslessMapMatrix*>(map_matrix_)
        ->GetMapCell(row, col)
        .map_cells[0];
  }

  /**@brief Get the min altitude of point cloud in the node. */
  inline float GetMinAltitude() const { return min_altitude_; }
  /**@brief Set the min altitude of point cloud in the node. */
  inline void SetMinAltitude(float altitude) { min_altitude_ = altitude; }

 protected:
  /**@brief The min altitude of point cloud in the node. */
  float min_altitude_;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
