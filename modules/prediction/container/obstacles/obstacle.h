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

#pragma once

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "modules/common/filters/digital_filter.h"
#include "modules/common/math/kalman_filter.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/prediction/proto/feature.pb.h"

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
  static std::unique_ptr<Obstacle>
  Create(const perception::PerceptionObstacle& perception_obstacle,
      const double timestamp, const int prediction_id);

  static std::unique_ptr<Obstacle>
  Create(const Feature& feature);

  /**
   * @brief Destructor
   */
  virtual ~Obstacle() = default;

  /**
   * @brief Insert a perception obstacle with its timestamp.
   * @param perception_obstacle The obstacle from perception.
   * @param timestamp The timestamp when the perception obstacle was detected.
   */
  bool Insert(const perception::PerceptionObstacle& perception_obstacle,
              const double timestamp, const int prediction_id);

  /**
   * @brief Insert a feature proto message.
   * @param feature proto message.
   */
  bool InsertFeature(const Feature& feature);

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

  bool ReceivedNewerMessage(const double timestamp) const;

  /**
   * @brief Get the ith feature from latest to earliest.
   * @param i The index of the feature.
   * @return The ith feature.
   */
  const Feature& feature(const size_t i) const;

  /**
   * @brief Get a pointer to the ith feature from latest to earliest.
   * @param i The index of the feature.
   * @return A pointer to the ith feature.
   */
  Feature* mutable_feature(const size_t i);

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
   * @brief Set nearby obstacles.
   */
  void SetNearbyObstacles();

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
  bool IsOnLane() const;

  /**
   * @brief Check if the obstacle can be ignored.
   * @return If the obstacle can be ignored.
   */
  bool ToIgnore();

  /**
   * @brief Check if the obstacle is near a junction.
   * @return If the obstacle is near a junction.
   */
  bool IsNearJunction();

  /**
   * @brief Check if the obstacle is a junction.
   * @param junction ID
   * @return If the obstacle is in a junction.
   */
  bool IsInJunction(const std::string& junction_id);

  /**
   * @brief Check if the obstacle is close to a junction exit.
   * @return If the obstacle is closed to a junction exit.
   */
  bool IsCloseToJunctionExit();

  /**
   * @brief Check if the obstacle has junction feature.
   * @return If the obstacle has junction feature.
   */
  bool HasJunctionFeatureWithExits();

  /**
   * @brief Build junction feature.
   */
  void BuildJunctionFeature();

  /**
   * @brief Build obstacle's lane graph
   */
  void BuildLaneGraph();

  /**
   * @brief Build obstacle's lane graph with lanes being ordered.
   *        This would be useful for lane-scanning algorithms.
   */
  void BuildLaneGraphFromLeftToRight();

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
  Obstacle() = default;

  void SetStatus(const perception::PerceptionObstacle& perception_obstacle,
                 double timestamp, Feature* feature);

  void UpdateStatus(Feature* feature);

  bool SetId(const perception::PerceptionObstacle& perception_obstacle,
             Feature* feature, const int prediction_id = -1);

  bool SetType(const perception::PerceptionObstacle& perception_obstacle,
               Feature* feature);

  void SetIsNearJunction(
      const perception::PerceptionObstacle& perception_obstacle,
      Feature* feature);

  void SetTimestamp(const perception::PerceptionObstacle& perception_obstacle,
                    const double timestamp, Feature* feature);

  void SetPolygonPoints(
      const perception::PerceptionObstacle& perception_obstacle,
      Feature* feature);

  void SetPosition(const perception::PerceptionObstacle& perception_obstacle,
                   Feature* feature);

  void SetVelocity(const perception::PerceptionObstacle& perception_obstacle,
                   Feature* feature);

  void AdjustHeadingByLane(Feature* feature);

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

  void SetLaneSequenceStopSign(LaneSequence* lane_sequence_ptr);

  void SetLanePoints(Feature* feature);

  void SetLanePoints(const Feature* feature, double lane_point_spacing,
                     LaneGraph* const lane_graph);

  void SetLaneSequencePath(LaneGraph* const lane_graph);

  void InitKFPedestrianTracker(const Feature& feature);

  void UpdateKFPedestrianTracker(const Feature& feature);

  void SetMotionStatus();

  void SetMotionStatusBySpeed();

  void InsertFeatureToHistory(const Feature& feature);

  void SetJunctionFeatureWithEnterLane(
      const std::string& enter_lane_id, Feature* const feature_ptr);

  void SetJunctionFeatureWithoutEnterLane(Feature* const feature_ptr);

  void DiscardOutdatedHistory();

 private:
  int id_ = -1;

  perception::PerceptionObstacle::Type type_ =
      perception::PerceptionObstacle::UNKNOWN_UNMOVABLE;

  std::deque<Feature> feature_history_;

  common::math::KalmanFilter<double, 6, 2, 0> kf_motion_tracker_;

  common::math::KalmanFilter<double, 2, 2, 4> kf_pedestrian_tracker_;

  std::vector<std::shared_ptr<const hdmap::LaneInfo>> current_lanes_;

  std::vector<Eigen::MatrixXf> rnn_states_;

  bool rnn_enabled_ = false;
};

}  // namespace prediction
}  // namespace apollo
