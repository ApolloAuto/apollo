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
 * @file object_table.h
 **/

#ifndef MODULES_PLANNING_COMMON_OBJECT_TABLE_H
#define MODULES_PLANNING_COMMON_OBJECT_TABLE_H

#include <memory>
#include <string>

#include "modules/planning/common/lru_cache.h"
#include "modules/planning/common/map_object.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

class ObjectTable {
 public:
  ObjectTable() :
      _obstacle_cache(FLAGS_object_table_obstacle_capacity),
      _map_object_cache(FLAGS_object_table_obstacle_capacity) {}

  bool get_obstacle(const uint32_t id, Obstacle** const obstacle);
  void put_obstacle(std::unique_ptr<Obstacle> obstacle);
  bool get_map_object(const std::string& id, MapObject** const map_object);
  void put_map_object(std::unique_ptr<MapObject>& map_object);

 private:
  LRUCache<uint32_t, std::unique_ptr<Obstacle>> _obstacle_cache;
  LRUCache<std::string, std::unique_ptr<MapObject>> _map_object_cache;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_OBJECT_TABLE_H
