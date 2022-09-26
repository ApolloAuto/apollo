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
#include <list>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "modules/common/filters/digital_filter.h"
#include "modules/common/math/kalman_filter.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/prediction/common/junction_analyzer.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/container/obstacles/obstacle_clusters.h"
#include "modules/common_msgs/prediction_msgs/feature.pb.h"
#include "modules/prediction/proto/prediction_conf.pb.h"
#include "modules/common_msgs/prediction_msgs/prediction_obstacle.pb.h"

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
  static std::unique_ptr<Obstacle> Create(
      const perception::PerceptionObstacle& perception_obstacle,
      const double timestamp, const int prediction_id,
      ObstacleClusters* clusters_ptr);

  static std::unique_ptr<Obstacle> Create(const Feature& feature,
                                          ObstacleClusters* clusters_ptr);

  Obstacle() = default;

  /**
   * @brief Destructor
   */
  virtual ~Obstacle() = default;

  void SetJunctionAnalyzer(JunctionAnalyzer* junction_analyzer) {
    junction_analyzer_ = junction_analyzer;
  }

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

  void ClearOldInformation();

  void TrimHistory(const size_t remain_size);

  /**
   * @brief Get the type of perception obstacle's type.
   * @return The type pf perception obstacle.
   */
  perception::PerceptionObstacle::Type type() const;

  bool IsPedestrian() const;

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

  bool ReceivedOlderMessage(const double timestamp) const;

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
   * @brief Get the earliest feature.
   * @return The earliest feature.
   */
  const Feature& earliest_feature() const;

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
   * @brief Check if the obstacle is still.
   * @return If the obstacle is still.
   */
  bool IsStill();

  /**
   * @brief Check if the obstacle is slow.
   * @return If the obstacle is slow.
   */
  bool IsSlow();

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
  bool IsInJunction(const std::string& junction_id) const;

  /**
   * @brief Check if the obstacle is close to a junction exit.
   * @return If the obstacle is closed to a junction exit.
   */
  bool IsCloseToJunctionExit() const;

  /**
   * @brief Check if the obstacle has junction feature.
   * @return If the obstacle has junction feature.
   */
  bool HasJunctionFeatureWithExits() const;

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
   * @brief Set the obstacle as caution level
   */
  void SetCaution();

  bool IsCaution() const;

  /**
   * @brief Set the obstacle as interactive obstacle.
   */
  void SetInteractiveTag();
  /**
   * @brief Set the obstacle as noninteractive obstacle.
   */
  void SetNonInteractiveTag();

  bool IsInteractiveObstacle() const;

  void SetEvaluatorType(const ObstacleConf::EvaluatorType& evaluator_type);

  void SetPredictorType(const ObstacleConf::PredictorType& predictor_type);

  const ObstacleConf& obstacle_conf() { return obstacle_conf_; }

  PredictionObstacle GeneratePredictionObstacle();

 private:
  void SetStatus(const perception::PerceptionObstacle& perception_obstacle,
                 double timestamp, Feature* feature);

  bool SetId(const perception::PerceptionObstacle& perception_obstacle,
             Feature* feature, const int prediction_id = -1);

  void SetType(const perception::PerceptionObstacle& perception_obstacle,
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

  void UpdateLaneBelief(Feature* feature);

  void SetCurrentLanes(Feature* feature);

  void SetNearbyLanes(Feature* feature);

  void SetSurroundingLaneIds(Feature* feature, const double radius);

  void SetLaneSequenceStopSign(LaneSequence* lane_sequence_ptr);

  /** @brief This functions updates the lane-points into the lane-segments
   *        based on the given lane_point_spacing.
   */
  void SetLanePoints(Feature* feature);
  void SetLanePoints(const Feature* feature, const double lane_point_spacing,
                     const uint64_t max_num_lane_point,
                     const bool is_bidirection, LaneGraph* const lane_graph);

  /** @brief This functions is mainly for lane-sequence kappa calculation.
   */
  void SetLaneSequencePath(LaneGraph* const lane_graph);

  void SetMotionStatus();

  void SetMotionStatusBySpeed();

  void InsertFeatureToHistory(const Feature& feature);

  void SetJunctionFeatureWithEnterLane(const std::string& enter_lane_id,
                                       Feature* const feature_ptr);

  void SetJunctionFeatureWithoutEnterLane(Feature* const feature_ptr);

  void DiscardOutdatedHistory();

  void GetNeighborLaneSegments(
      std::shared_ptr<const apollo::hdmap::LaneInfo> center_lane_info,
      bool is_left, int recursion_depth,
      std::list<std::string>* const lane_ids_ordered,
      std::unordered_set<std::string>* const existing_lane_ids);

  bool HasJunctionExitLane(
      const LaneSequence& lane_sequence,
      const std::unordered_set<std::string>& exit_lane_id_set);

  void SetClusters(ObstacleClusters* clusters_ptr);

 private:
  int id_ = FLAGS_ego_vehicle_id;

  perception::PerceptionObstacle::Type type_ =
      perception::PerceptionObstacle::UNKNOWN_UNMOVABLE;

  std::deque<Feature> feature_history_;

  std::vector<std::shared_ptr<const hdmap::LaneInfo>> current_lanes_;

  ObstacleConf obstacle_conf_;

  ObstacleClusters* clusters_ptr_ = nullptr;
  JunctionAnalyzer* junction_analyzer_ = nullptr;
};

}  // namespace prediction
}  // namespace apollo
