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
 * @brief This file provides the implementation of the class
 * "QuadTilesMaker".
 */
#include "modules/tools/navi_generator/backend/util/quad_tiles_maker.h"

#include "modules/common/log.h"

namespace apollo {
namespace navi_generator {
namespace util {

QuadTilesMaker::QuadTilesMaker() { ; }

bool QuadTilesMaker::MakeQuadTile(const double lat, const double lon,
                                  const double altitude,
                                  const std::size_t level,
                                  QuadTile* const quad_tile) {
  CHECK_NOTNULL(quad_tile);
  if (level < 1) {
    AERROR << "Cannot handle desired level " << level << " (< 1)";
    return false;
  }
  if (level > 32) {
    AERROR << "Cannot handle desired level " << level << " (> 32)";
    return false;
  }

  Position pos;
  if (!MakePosition(lat, lon, altitude, &pos)) {
    return false;
  }

  std::uint64_t id;
  double center_lat;
  double center_lon;
  std::size_t current_level = 0;
  double lat_adjustment = 45.0;
  double lon_adjustment = 90.0;
  std::uint64_t bit = 0x8000000000000000;

  while (current_level < level) {
    if (lat >= center_lat) {
      // north
      center_lat += lat_adjustment;
    } else {
      // south
      id |= bit;
      center_lat -= lat_adjustment;
    }

    bit /= 2;

    if (lon < center_lon) {
      // west
      center_lon -= lon_adjustment;
    } else {
      // east
      id |= bit;
      center_lon += lon_adjustment;
    }

    bit /= 2;
    lat_adjustment /= 2.0;
    lon_adjustment /= 2.0;
    current_level++;
  }

  quad_tile->id = id;
  quad_tile->level = level;
  quad_tile->position.lat = pos.lat;
  quad_tile->position.lon = pos.lon;
  quad_tile->position.altitude = pos.altitude;
  return true;
}

bool QuadTilesMaker::IdAsString(const std::size_t level,
                                QuadTile* const quad_tile,
                                std::string* const id_string) {
  CHECK_NOTNULL(quad_tile);
  CHECK_NOTNULL(id_string);
  if (level < 1) {
    AERROR << "Cannot handle desired level " << level << " (< 1)";
    return false;
  } else if (level > quad_tile->level) {
    AERROR << "The desired level " << level << " is greater than QuadTile ("
           << quad_tile->level << ")";
    return false;
  } else {
    // nothing
  }

  auto id = quad_tile->id;
  *id_string = "";

  for (std::size_t i = 0; i < level; i++) {
    id_string->append(1, static_cast<char>(97 + (id >> 62)));
    id = id << 2;
  }

  return true;
}

bool QuadTilesMaker::IdAsUint32(const std::size_t level,
                                QuadTile* const quad_tile,
                                std::uint32_t* const id_uint32) {
  CHECK_NOTNULL(quad_tile);
  CHECK_NOTNULL(id_uint32);
  if (level < 1) {
    AERROR << "Cannot handle desired level " << level << " (< 1)";
    return false;
  } else if (level > 16) {
    AERROR << "Cannot handle desired level " << level << " (> 16)";
    return false;
  } else if (level > quad_tile->level) {
    AERROR << "The desired level " << level << " is greater than QuadTile ("
           << quad_tile->level << ")";
    return false;
  } else {
    // nothing
  }

  *id_uint32 = quad_tile->id >> 32;
  return true;
}

bool QuadTilesMaker::MakePosition(double lat, double lon, double altitude,
                                  Position* const pos) {
  CHECK_NOTNULL(pos);
  if (lat < -90.0 || lat > 90.0 || lon <= -180.0 || lon > 180.0) {
    AERROR << "The latitude: " << lat << ", longitude: " << lon
           << ", altitude: " << altitude << " is out of range.";
    return false;
  }

  pos->lat = lat;
  pos->lon = lon;
  pos->altitude = altitude;

  return true;
}

}  // namespace util
}  // namespace navi_generator
}  // namespace apollo
