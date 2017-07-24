/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
 * @file object_table.cc
 **/

#include "modules/planning/common/object_table.h"

namespace apollo {
namespace planning {

bool ObjectTable::get_obstacle(const uint32_t id, Obstacle** const obstacle) {
  auto *obstacle_from_cache = _obstacle_cache.get(id);
  if (!obstacle_from_cache) {
    return false;
  }

  *obstacle = obstacle_from_cache->get();
  return true;
}

void ObjectTable::put_obstacle(std::unique_ptr<Obstacle> obstacle) {
  _obstacle_cache.put(obstacle->Id(), std::move(obstacle));
}

bool ObjectTable::get_map_object(const std::string& id,
                                 MapObject** const map_object) {
  auto *map_object_from_cache = _map_object_cache.get(id);
  if (!map_object_from_cache) {
    return false;
  }

  *map_object = map_object_from_cache->get();

  return true;
}

void ObjectTable::put_map_object(std::unique_ptr<MapObject>& map_object) {
  _map_object_cache.put(map_object->Id(), std::move(map_object));
}

}  // namespace planning
}  // namespace apollo
