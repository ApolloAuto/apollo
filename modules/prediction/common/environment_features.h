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

#ifndef MODULES_PREDICTION_COMMON_ENVIRONMENT_FEATURES_H_
#define MODULES_PREDICTION_COMMON_ENVIRONMENT_FEATURES_H_

#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace apollo {
namespace prediction {

class EnvironmentFeatures {
 public:
  EnvironmentFeatures() = default;

  virtual ~EnvironmentFeatures() = default;

  void set_ego_velocity(const double ego_velocity);

  double get_ego_velocity() const;

  void set_ego_acceleration(const double ego_acceleration);

  double get_ego_acceleration() const;

  void set_ego_heading(const double ego_heading);

  double get_ego_heading() const;

  bool has_ego_lane() const;

  void reset_ego_lane();

  void SetEgoLane(const std::string& lane_id, const double lane_s);

  std::pair<std::string, double> GetEgoLane() const;

  bool has_left_neighbor_lane() const;

  void reset_left_neighbor_lane();

  void SetLeftNeighborLane(const std::string& lane_id, const double lane_s);

  std::pair<std::string, double> GetLeftNeighborLane() const;

  bool has_right_neighbor_lane() const;

  void reset_right_neighbor_lane();

  void SetRightNeighborLane(const std::string& lane_id, const double lane_s);

  std::pair<std::string, double> GetRightNeighborLane() const;

  bool has_front_junction() const;

  void reset_front_junction();

  void SetFrontJunction(const std::string& junction_id, const double dist);

  std::pair<std::string, double> GetFrontJunction() const;

  void AddObstacleId(const int obstacle_id);

  const std::vector<int>& get_obstacle_ids() const;

 private:
  double ego_velocity_ = 0.0;

  double ego_acceleration_ = 0.0;

  double ego_heading_ = 0.0;

  bool has_ego_lane_ = false;

  std::string ego_lane_id_;

  double ego_lane_s_ = 0.0;

  bool has_left_neighbor_lane_ = false;

  std::string left_neighbor_lane_id_;

  double left_neighbor_lane_s_ = 0.0;

  bool has_right_neighbor_lane_ = false;

  std::string right_neighbor_lane_id_;

  double right_neighbor_lane_s_ = 0.0;

  bool has_front_junction_ = false;

  std::string front_junction_id_;

  double dist_to_front_junction_ = 0.0;

  std::vector<int> obstacle_ids_;
};

}  // namespace prediction
}  // namespace apollo

#endif /* MODULES_PREDICTION_COMMON_ENVIRONMENT_FEATURES_H_ */
