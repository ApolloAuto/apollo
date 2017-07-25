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
 * @file
 * @brief Obstacle
 */

#ifndef MODULES_PREDICTION_CONTAINER_OBSTACLES_OBSTACLE_H_
#define MODULES_PREDICTION_CONTAINER_OBSTACLES_OBSTACLE_H_

#include <deque>
#include <unordered_map>
#include <vector>
#include <string>
#include <mutex>

#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/prediction/proto/feature.pb.h"
#include "modules/common/proto/error_code.pb.h"
#include "modules/common/math/kalman_filter.h"
#include "modules/map/hdmap/hdmap_common.h"

namespace apollo {
namespace prediction {

class Obstacle {
 public:
  Obstacle();

  virtual ~Obstacle();

  void Insert(
    const apollo::perception::PerceptionObstacle& perception_obstacle,
    const double timestamp);

  int id() const;

  double timestamp() const;

  const Feature& feature(size_t i);

  Feature* mutable_feature(size_t i);

  const Feature& latest_feature();

  Feature* mutable_latest_feature();

  size_t history_size() const;

  const apollo::common::math::KalmanFilter<double, 4, 2, 0>& kf_lane_tracker(
      const std::string& lane_id);

  const apollo::common::math::KalmanFilter<double, 6, 2, 0>& kf_motion_tracker();

  bool IsOnLane();

 private:
  apollo::common::ErrorCode SetId(
      const apollo::perception::PerceptionObstacle& perception_obstacle,
      Feature* feature);

  apollo::common::ErrorCode SetType(
      const apollo::perception::PerceptionObstacle& perception_obstacle);

  void SetTimestamp(
      const apollo::perception::PerceptionObstacle& perception_obstacle,
      const double timestamp,
      Feature* feature);

  void SetPosition(
      const apollo::perception::PerceptionObstacle& perception_obstacle,
      Feature* feature);

  void SetVelocity(
      const apollo::perception::PerceptionObstacle& perception_obstacle,
      Feature* feature);

  void SetAcceleration(Feature* feature);

  void SetTheta(
      const apollo::perception::PerceptionObstacle& perception_obstacle,
      Feature* feature);

  void SetLengthWidthHeight(
      const apollo::perception::PerceptionObstacle& perception_obstacle,
      Feature* feature);

  void InitKFMotionTracker(Feature* feature);

  void UpdateKFMotionTracker(Feature* feature);

  void UpdateMotionBelief(Feature* feature);

  void InitKFLaneTracker(const std::string& lane_id, const double beta);

  void UpdateKFLaneTrackers(Feature* feature);

  void UpdateKFLaneTracker(
      const std::string& lane_id,
      const double lane_s, const double lane_l,
      const double lane_speed, const double lane_acc,
      const double timestamp, const double beta);

  void UpdateLaneBelief(Feature* feature);

  void SetCurrentLanes(Feature* feature);

  void SetNearbyLanes(Feature* feature);

  void SetLaneGraphFeature(Feature* feature);

  void SetLanePoints(Feature* feature);

  void SetMotionStatus();

  void InsertFeatureToHistory(Feature* feature);

  void Trim();

 private:
  int id_;
  apollo::perception::PerceptionObstacle::Type type_;
  std::deque<Feature> feature_history_;
  apollo::common::math::KalmanFilter<double, 6, 2, 0> kf_motion_tracker_;
  bool kf_motion_tracker_enabled_;
  std::unordered_map<std::string,
      apollo::common::math::KalmanFilter<double, 4, 2, 0>> kf_lane_trackers_;
  std::vector<const apollo::hdmap::LaneInfo*> current_lanes_;
  static std::mutex mutex_;
};

}  // namespace prediction
}  // namespace apollo

#endif  // MODULES_PREDICTION_CONTAINER_OBSTACLES_OBSTACLE_H_
