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
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "Eigen/Dense"

#include "modules/common/filters/digital_filter.h"
#include "modules/common/proto/error_code.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/prediction/proto/feature.pb.h"

#include "modules/common/math/kalman_filter.h"
#include "modules/map/hdmap/hdmap_common.h"

/**
 * @namespace apollo::prediction
 * @brief apollo::prediction
 */
namespace apollo {
namespace prediction {

/**
 * @class Obstacle
 * @brief Prediction obstacle.
 */
class Obstacle {
 public:
  /**
   * @brief Constructor
   */
  Obstacle();

  /**
   * @brief Destructor
   */
  virtual ~Obstacle() = default;

  /**
   * @brief Insert a perception obstacle with its timestamp.
   * @param perception_obstacle The obstacle from perception.
   * @param timestamp The timestamp when the perception obstacle was detected.
   */
  void Insert(const perception::PerceptionObstacle& perception_obstacle,
              const double timestamp);

  /**
   * @brief Get the type of perception obstacle's type.
   * @return The type pf perception obstacle.
   */
  perception::PerceptionObstacle::Type type() const;

  /**
   * @brief Get the obstacle's ID.
   * @return The obstacle's ID.
   */
  int id() const;

  /**
   * @brief Get the obstacle's timestamp.
   * @return The obstacle's timestamp.
   */
  double timestamp() const;

  /**
   * @brief Get the ith feature from latest to earliest.
   * @param i The index of the feature.
   * @return The ith feature.
   */
  const Feature& feature(size_t i) const;

  /**
   * @brief Get a pointer to the ith feature from latest to earliest.
   * @param i The index of the feature.
   * @return A pointer to the ith feature.
   */
  Feature* mutable_feature(size_t i);

  /**
   * @brief Get the latest feature.
   * @return The latest feature.
   */
  const Feature& latest_feature() const;

  /**
   * @brief Get a pointer to the latest feature.
   * @return A pointer to the latest feature.
   */
  Feature* mutable_latest_feature();

  /**
   * @brief Get the number of historical features.
   * @return The number of historical features.
   */
  size_t history_size() const;

  /**
   * @brief Get the motion Kalman filter.
   * @return The motion Kalman filter.
   */
  const common::math::KalmanFilter<double, 6, 2, 0>& kf_motion_tracker() const;

  /**
   * @brief Get the pedestrian Kalman filter.
   * @return The pedestrian Kalman filter.
   */
  const common::math::KalmanFilter<double, 2, 2, 4>& kf_pedestrian_tracker()
      const;

  /**
   * @brief Check if the obstacle is still.
   * @return If the obstacle is still.
   */
  bool IsStill();

  /**
   * @brief Check if the obstacle is on any lane.
   * @return If the obstacle is on any lane.
   */
  bool IsOnLane();

  /**
   * @brief Check if the obstacle is near a junction.
   * @return If the obstacle is near a junction.
   */
  bool IsNearJunction();

  /**
   * @brief Set RNN state
   * @param RNN state matrix
   */
  void SetRNNStates(const std::vector<Eigen::MatrixXf>& rnn_states);

  /**
   * @brief Get RNN state
   * @param A pointer to RNN state matrix
   */
  void GetRNNStates(std::vector<Eigen::MatrixXf>* rnn_states);

  /**
   * @brief Initialize RNN state
   */
  void InitRNNStates();

  /**
   * @brief Check if RNN is enabled
   * @return True if RNN is enabled
   */
  bool RNNEnabled() const;

 private:
  void SetStatus(const perception::PerceptionObstacle& perception_obstacle,
                 double timestamp, Feature* feature);

  void UpdateStatus(Feature* feature);

  common::ErrorCode SetId(
      const perception::PerceptionObstacle& perception_obstacle,
      Feature* feature);

  common::ErrorCode SetType(
      const perception::PerceptionObstacle& perception_obstacle,
      Feature* feature);

  void SetTimestamp(const perception::PerceptionObstacle& perception_obstacle,
                    const double timestamp, Feature* feature);

  void SetPosition(const perception::PerceptionObstacle& perception_obstacle,
                   Feature* feature);

  void SetVelocity(const perception::PerceptionObstacle& perception_obstacle,
                   Feature* feature);

  void UpdateVelocity(const double theta, double* velocity_x,
                      double* velocity_y, double* velocity_heading,
                      double* speed);

  void SetAcceleration(Feature* feature);

  void SetTheta(const perception::PerceptionObstacle& perception_obstacle,
                Feature* feature);

  void SetLengthWidthHeight(
      const perception::PerceptionObstacle& perception_obstacle,
      Feature* feature);

  void InitKFMotionTracker(const Feature& feature);

  void UpdateKFMotionTracker(const Feature& feature);

  void UpdateLaneBelief(Feature* feature);

  void SetCurrentLanes(Feature* feature);

  void SetNearbyLanes(Feature* feature);

  void SetLaneGraphFeature(Feature* feature);

  void SetLanePoints(Feature* feature);

  void SetLaneSequencePath(LaneGraph* const lane_graph);

  void InitKFPedestrianTracker(const Feature& feature);

  void UpdateKFPedestrianTracker(const Feature& feature);

  void SetMotionStatus();

  void SetMotionStatusBySpeed();

  void InsertFeatureToHistory(const Feature& feature);

  void Trim();

 private:
  int id_ = -1;
  perception::PerceptionObstacle::Type type_ =
      perception::PerceptionObstacle::UNKNOWN_UNMOVABLE;
  std::deque<Feature> feature_history_;
  common::math::KalmanFilter<double, 6, 2, 0> kf_motion_tracker_;
  common::math::KalmanFilter<double, 2, 2, 4> kf_pedestrian_tracker_;
  common::DigitalFilter heading_filter_;
  std::vector<std::shared_ptr<const hdmap::LaneInfo>> current_lanes_;
  std::vector<Eigen::MatrixXf> rnn_states_;
  bool rnn_enabled_ = false;
};

}  // namespace prediction
}  // namespace apollo

#endif  // MODULES_PREDICTION_CONTAINER_OBSTACLES_OBSTACLE_H_
