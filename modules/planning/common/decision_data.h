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

enum class VirtualObjectType {
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
  explicit DecisionData(
      const prediction::PredictionObstacles& prediction_obstacles,
      const ReferenceLine& reference_line);
  ~DecisionData() = default;

 public:
  PathObstacle* GetObstacleById(const std::string& id);
  std::vector<PathObstacle*> GetObstacleByType(const VirtualObjectType& type);
  std::vector<std::string> GetObstacleIdByType(const VirtualObjectType& type);
  const std::vector<PathObstacle*>& GetStaticObstacle() const;
  const std::vector<PathObstacle*>& GetDynamicObstacle() const;
  const std::vector<PathObstacle*>& GetVirtualObstacle() const;
  const std::vector<PathObstacle*>& GetPracticalObstacle() const;
  const std::vector<PathObstacle*>& GetAllObstacle() const;

 public:
  bool CreateVirtualObstacle(const ReferencePoint& point,
                             const VirtualObjectType& type,
                             std::string* const id);
  bool CreateVirtualObstacle(const double point_s,
                             const VirtualObjectType& type,
                             std::string* const id);

 private:
  bool IsValidTrajectory(const prediction::Trajectory& trajectory);
  bool IsValidTrajectoryPoint(const common::TrajectoryPoint& point);

 private:
  std::vector<PathObstacle*> static_obstacle_;
  std::vector<PathObstacle*> dynamic_obstacle_;
  std::vector<PathObstacle*> virtual_obstacle_;
  std::vector<PathObstacle*> practical_obstacle_;
  std::vector<PathObstacle*> all_obstacle_;

 private:
  const ReferenceLine& reference_line_;
  std::list<std::unique_ptr<Obstacle>> obstacles_;
  std::list<std::unique_ptr<PathObstacle>> path_obstacles_;
  std::unordered_map<std::string, PathObstacle*> obstacle_map_;
  std::unordered_map<VirtualObjectType, std::vector<std::string>, EnumClassHash>
      virtual_obstacle_id_map_;
  std::mutex mutex_;
  std::mutex transaction_mutex_;
};

}  // namespace planning
}  // namespace apollo
