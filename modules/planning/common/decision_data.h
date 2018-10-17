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

#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "modules/prediction/proto/prediction_obstacle.pb.h"

#include "modules/planning/common/path_obstacle.h"
#include "modules/planning/reference_line/reference_line.h"

namespace apollo {
namespace planning {

enum VirtualObjectType {
  VIRTUAL_OBJECT_TYPE_DESTINATION = 0,
  VIRTUAL_OBJECT_TYPE_CROSSWALK = 1,
  VIRTUAL_OBJECT_TYPE_TRAFFIC_LIGHT = 2,
  VIRTUAL_OBJECT_TYPE_CLEAR_ZONE = 3,
  VIRTUAL_OBJECT_TYPE_REROUTE = 4,
  VIRTUAL_OBJECT_TYPE_DECISION_JUMP = 5,
  VIRTUAL_OBJECT_TYPE_PRIORITY = 6
};

class DecisionData {
 public:
  explicit DecisionData(const prediction::PredictionObstacles obstacles,
                        const ReferenceLine& reference_line);
  ~DecisionData() = default;

 public:
  bool GetObstacleById(const std::string& id,
                       PathObstacle* const obstacle) const;
  bool GetObstacleByType(const VirtualObjectType& type,
                         std::vector<PathObstacle*>* const obstacles) const;
  bool GetObstacleIdByType(const VirtualObjectType& type,
                         std::vector<std::string>* const ids) const;
  const std::vector<PathObstacle*>& GetStaticObstacle() const;
  const std::vector<PathObstacle*>& GetDynamicObstacle() const;
  const std::vector<PathObstacle*>& GetVirtualObstacle() const;
  const std::vector<PathObstacle*>& GetPracticalObstacle() const;
  const std::vector<PathObstacle*>& GetAllObstacle() const;
  void Update();

 public:
  bool CreateVirtualObstacle(const ReferencePoint& point,
                             const VirtualObjectType& type,
                             std::string* const id);
  bool CreateVirtualObstacle(const double point_s,
                             const VirtualObjectType& type,
                             std::string* const id);

 private:
  std::vector<PathObstacle*> static_obstacle_;
  std::vector<PathObstacle*> dynamic_obstacle_;
  std::vector<PathObstacle*> virtual_obstacle_;
  std::vector<PathObstacle*> practical_obstacle_;
  std::vector<PathObstacle*> all_obstacle_;

 private:
  const ReferenceLine& reference_line_;
  std::list<std::unique_ptr<PathObstacle>> obstacle_;
  std::unordered_map<std::string, PathObstacle*> obstacle_map_;
  std::unordered_map<VirtualObjectType,
        std::vector<std::string>, std::hash<int>> virtual_object_id_map_;
  std::mutex mutex_;
};

}  // namespace planning
}  // namespace apollo

