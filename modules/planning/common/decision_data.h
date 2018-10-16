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

namespace apollo {
namespace planning {

enum VirtualObjectType {
  DESTINATION = 0,
  CROSSWALK = 1,
  TRAFFIC_LIGHT = 2,
  CLEAR_ZONE = 3,
  REROUTE = 4,
  DECISION_JUMP = 5,
  PRIORITY = 6
};

struct EnumClassHash {
  template <typename T>
  std::size_t operator()(T t) const {
      return static_cast<std::size_t>(t);
  }
};

class DecisionData {
 public:
  explicit DecisionData(const prediction::PredictionObstacles obstacles,
                        const ReferenceLine& reference_line);
  ~DecisionData() = default;
 public:
  bool GetObstacleById(const str::string& id,
                       PathObstacle* const obstacle) const;
  bool GetObstacleByType(const VirtualObjectType& type,
                         std::vector<std::string>* const ids) const;
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
  std::vector<PathObstacle*> all_obstacle;`
 private:
  const ReferenceLine& reference_line_;
  std::list<PathObstacle> obstacle_;
  std::unordered_map<std::string, std::unique_ptr<PathObstacle>> obstacle_map_;
  std::unordered_map<VirtualObjectType,
        std::vector<std::string>, EnumClassHash> virtual_object_id_map_;
  std::mutex mutex_;
};

}  // namespace planning
}  // namespace apollo

