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

#include <vector>

#include "Eigen/Core"

#include "modules/localization/msf/local_pyramid_map/base_map/base_map_node.h"
#include "modules/localization/msf/local_pyramid_map/pyramid_map/pyramid_map_matrix.h"

namespace apollo {
namespace localization {
namespace msf {
namespace pyramid_map {

class PyramidMapNode : public BaseMapNode {
 public:
  PyramidMapNode();
  ~PyramidMapNode();

 public:
  virtual void Init(const BaseMapConfig* map_config);
  virtual void Init(const BaseMapConfig* map_config, const MapNodeIndex& index,
                    bool create_map_cells = true);

  /**@brief Propagate the data to the coarse resolution by check. */
  void BottomUpSafe();

  /**@brief Propagate the data to the coarse resolution.
   * only update count, intensity, intensity var and altitude
   */
  void BottomUpBase();

  /**@brief Add the value of a pixel in the map node if the pixel in the node.
   * @param <coordinate> The 3D global coordinate.
   * @param <intensity> The reflectance intensity.
   * @param <return> True, if pixel in the bound of the node, else False.
   * */
  bool AddValueIfInBound(const Eigen::Vector3d& coordinate,
                         unsigned char intensity, unsigned int level = 0);
  /**@brief Add the value of a pixel in the map node if the pixel in the node.
   * @param <coordinates> The 3D global coordinates.
   * @param <intensities> The reflectance intensities.
   * */
  void AddValueIfInBound(const std::vector<Eigen::Vector3d>& coordinates,
                         const std::vector<unsigned char>& intensity,
                         unsigned int level = 0);
  /**@brief Given the global coordinate, get the local 2D coordinate of the map
   * cell matrix.
   * <return> If global coordinate (x, y) belongs to this map node. */
  bool GetCoordinate(const Eigen::Vector2d& coordinate, unsigned int level,
                     unsigned int* x, unsigned int* y) const;
  /**@brief Given the local 2D coordinate, return the global coordinate. */
  Eigen::Vector2d GetCoordinate(unsigned int level, unsigned int x,
                                unsigned int y) const;

  virtual bool GetCoordinate(const Eigen::Vector2d& coordinate, unsigned int* x,
                             unsigned int* y) const;
  virtual bool GetCoordinate(const Eigen::Vector3d& coordinate, unsigned int* x,
                             unsigned int* y) const;

  virtual Eigen::Vector2d GetCoordinate(unsigned int x, unsigned int y) const;

  /**@brief Given the 3D global coordinate,
   * get the map cell intensity with check. */
  float GetIntensitySafe(const Eigen::Vector3d& coordinate,
                         unsigned int level = 0) const;
  /**@brief Given the 3D global coordinate,
   * get the map cell variance of the intensity with check. */
  float GetIntensityVarSafe(const Eigen::Vector3d& coordinate,
                            unsigned int level = 0) const;
  /**@brief Given the 3D global coordinate,
   * get the map cell's average altitude with check. */
  float GetAltitudeSafe(const Eigen::Vector3d& coordinate,
                        unsigned int level = 0) const;
  /**@brief Given the 3D global coordinate,
   * get the map cell's variance of the altitude with check. */
  float GetAltitudeVarSafe(const Eigen::Vector3d& coordinate,
                           unsigned int level = 0) const;
  /**@brief Given the 3D global coordinate,
   * get the map cell's average ground altitude with check. */
  float GetGroundAltitudeSafe(const Eigen::Vector3d& coordinate,
                              unsigned int level = 0) const;
  /**@brief Given the 3D global coordinate,
   * get the map cell's count of the samples with check. */
  unsigned int GetCountSafe(const Eigen::Vector3d& coordinate,
                            unsigned int level = 0) const;
  /**@brief Given the 3D global coordinate,
   * get the map cell's count of the ground samples with check. */
  unsigned int GetGroundCountSafe(const Eigen::Vector3d& coordinate,
                                  unsigned int level = 0) const;

  /**@brief Given the 3D global coordinate,
   * get the map cell intensity without check. */
  float GetIntensity(const Eigen::Vector3d& coordinate,
                     unsigned int level = 0) const;
  /**@brief Given the 3D global coordinate,
   * get the map cell variance of the intensity without check. */
  float GetIntensityVar(const Eigen::Vector3d& coordinate,
                        unsigned int level = 0) const;
  /**@brief Given the 3D global coordinate,
   * get the map cell's average altitude without check. */
  float GetAltitude(const Eigen::Vector3d& coordinate,
                    unsigned int level = 0) const;
  /**@brief Given the 3D global coordinate,
   * get the map cell's variance of the altitude without check. */
  float GetAltitudeVar(const Eigen::Vector3d& coordinate,
                       unsigned int level = 0) const;
  /**@brief Given the 3D global coordinate,
   * get the map cell's average ground altitude without check. */
  float GetGroundAltitude(const Eigen::Vector3d& coordinate,
                          unsigned int level = 0) const;
  /**@brief Given the 3D global coordinate,
   * get the map cell's count of the samples without check. */
  unsigned int GetCount(const Eigen::Vector3d& coordinate,
                        unsigned int level = 0) const;
  /**@brief Given the 3D global coordinate,
   * get the map cell's count of the ground samples without check. */
  unsigned int GetGroundCount(const Eigen::Vector3d& coordinate,
                              unsigned int level = 0) const;

  /**@brief Compute mean intensity. */
  double ComputeMeanIntensity(unsigned int level = 0);

 private:
  std::vector<float> resolutions_mr_;
};

}  // namespace pyramid_map
}  // namespace msf
}  // namespace localization
}  // namespace apollo
