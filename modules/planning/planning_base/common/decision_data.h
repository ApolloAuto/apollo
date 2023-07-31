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

#include "modules/common_msgs/prediction_msgs/prediction_obstacle.pb.h"
#include "modules/common/math/box2d.h"
#include "modules/planning/planning_base/common/obstacle.h"
#include "modules/planning/planning_base/reference_line/reference_line.h"

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
  size_t operator()(T t) const {
    return static_cast<size_t>(t);
  }
};

class DecisionData {
 public:
  DecisionData(const prediction::PredictionObstacles& prediction_obstacles,
               const ReferenceLine& reference_line);
  ~DecisionData() = default;

 public:
  Obstacle* GetObstacleById(const std::string& id);
  std::vector<Obstacle*> GetObstacleByType(const VirtualObjectType& type);
  std::unordered_set<std::string> GetObstacleIdByType(
      const VirtualObjectType& type);
  const std::vector<Obstacle*>& GetStaticObstacle() const;
  const std::vector<Obstacle*>& GetDynamicObstacle() const;
  const std::vector<Obstacle*>& GetVirtualObstacle() const;
  const std::vector<Obstacle*>& GetPracticalObstacle() const;
  const std::vector<Obstacle*>& GetAllObstacle() const;

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
  bool CreateVirtualObstacle(const common::math::Box2d& obstacle_box,
                             const VirtualObjectType& type,
                             std::string* const id);

 private:
  std::vector<Obstacle*> static_obstacle_;
  std::vector<Obstacle*> dynamic_obstacle_;
  std::vector<Obstacle*> virtual_obstacle_;
  std::vector<Obstacle*> practical_obstacle_;
  std::vector<Obstacle*> all_obstacle_;

 private:
  const ReferenceLine& reference_line_;
  std::list<std::unique_ptr<Obstacle>> obstacles_;
  std::unordered_map<std::string, Obstacle*> obstacle_map_;
  std::unordered_map<VirtualObjectType, std::unordered_set<std::string>,
                     EnumClassHash>
      virtual_obstacle_id_map_;
  std::mutex mutex_;
  std::mutex transaction_mutex_;
};

}  // namespace planning
}  // namespace apollo
