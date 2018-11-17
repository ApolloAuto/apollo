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

/**
 * @file
 * @brief This file provides the declaration of the class "QuadTilesMaker" which
 * implement the QuadTile hierarchichal binning algorithm described by the
 * OpenStreetMap project.
 */

#ifndef MODULES_TOOLS_NAVI_GENERATOR_BACKEND_UTIL_QUAD_TILES_MAKER_H_
#define MODULES_TOOLS_NAVI_GENERATOR_BACKEND_UTIL_QUAD_TILES_MAKER_H_

#include <cstdio>
#include <string>

/**
 * @namespace apollo::navi_generator::util
 * @brief apollo::navi_generator::util
 */
namespace apollo {
namespace navi_generator {
namespace util {

struct Position {
  double lat;
  double lon;
  double altitude;
};

struct QuadTile {
  std::uint64_t id;
  std::size_t level;
  Position position;
};

class QuadTilesMaker {
 public:
  QuadTilesMaker();
  ~QuadTilesMaker() = default;

 public:
  /**
   * @brief Create a QuadTile.
   * @param lat The latitude.
   * @param lon The longitude.
   * @param altitude The altitude.
   * @param level The zoom level desired.
   * @param quad_tile Output value, the QuadTile to be created.
   * @return  Return true for success.
   */
  bool MakeQuadTile(const double lat, const double lon, const double altitude,
                    const std::size_t level, QuadTile* const quad_tile);
  /**
   * @brief Covert QuadTile's id to a string.
   * @param level The zoom level desired.
   * @param quad_tile The QuadTile to be converted.
   * @param id_string The output value of QuadTile's id as a string.
   * @return  Return true for success.
   */
  bool IdAsString(const std::size_t level, QuadTile* const quad_tile,
                  std::string* const id_string);
  /**
   * @brief Covert QuadTile's id to a unsinged int.
   * @param level The zoom level desired.
   * @param quad_tile The QuadTile to be converted.
   * @param id_uint32 The output value of QuadTile's id as a unsigned int.
   * @return  Return true for success.
   */
  bool IdAsUint32(const std::size_t level, QuadTile* const quad_tile,
                  std::uint32_t* const id_uint32);

 private:
  bool MakePosition(double lat, double lon, double altitude,
                    Position* const pos);
};

}  // namespace util
}  // namespace navi_generator
}  // namespace apollo

#endif  // MODULES_TOOLS_NAVI_GENERATOR_BACKEND_UTIL_QUAD_TILES_MAKER_H_
