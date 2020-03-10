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
 * @brief ADC trajectory container
 */

#pragma once

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "modules/planning/proto/planning.pb.h"
#include "modules/prediction/common/prediction_map.h"
#include "modules/prediction/container/container.h"
#include "modules/prediction/proto/lane_graph.pb.h"

namespace apollo {
namespace prediction {

class ADCTrajectoryContainer : public Container {
 public:
  /**
   * @brief Constructor
   */
  ADCTrajectoryContainer();

  /**
   * @brief Destructor
   */
  virtual ~ADCTrajectoryContainer() = default;

  /**
   * @brief Insert a data message into the container
   * @param Data message to be inserted in protobuf
   */
  void Insert(const ::google::protobuf::Message& message) override;

  /**
   * @brief Get the right-of-way status of ADC
   * @return The right-of-way status of ADC
   */
  bool IsProtected() const;

  /**
   * @brief Check if a point is in the first junction of the adc trajectory
   * @param Point
   * @return True if the point is in the first junction of the adc trajectory
   */
  bool IsPointInJunction(const common::PathPoint& point) const;

  /**
   * @brief Has overlap with ADC trajectory
   * @return True if a target lane sequence has overlap with ADC trajectory
   */
  bool HasOverlap(const LaneSequence& lane_sequence) const;

  /**
   * @brief Set ADC position
   */
  void SetPosition(const common::math::Vec2d& position);

  /**
   * @brief Get ADC junction
   * @return A pointer to ADC junction information
   */
  std::shared_ptr<const hdmap::JunctionInfo> ADCJunction() const;

  /**
   * @brief Compute ADC's distance to junction
   * @return ADC's distance to junction
   */
  double ADCDistanceToJunction() const;

  /**
   * @brief Get ADC planning trajectory
   * @return ADC planning trajectory
   */
  const planning::ADCTrajectory& adc_trajectory() const;

  /**
   * @brief Determine if a lane ID is in the reference line
   * @return The lane ID to be investigated
   */
  bool IsLaneIdInReferenceLine(const std::string& lane_id) const;

  bool IsLaneIdInTargetReferenceLine(const std::string& lane_id) const;

  const std::vector<std::string>& GetADCLaneIDSequence() const;

  const std::vector<std::string>& GetADCTargetLaneIDSequence() const;

  void SetJunction(const std::string& junction_id, const double distance);

 private:
  void SetJunctionPolygon();

  void SetLaneSequence();

  void SetTargetLaneSequence();

  std::string ToString(const std::unordered_set<std::string>& lane_ids);

  std::string ToString(const std::vector<std::string>& lane_ids);

 private:
  planning::ADCTrajectory adc_trajectory_;
  common::math::Polygon2d adc_junction_polygon_;
  std::shared_ptr<const hdmap::JunctionInfo> adc_junction_info_ptr_;
  double s_dist_to_junction_;
  std::unordered_set<std::string> adc_lane_ids_;
  std::vector<std::string> adc_lane_seq_;
  std::unordered_set<std::string> adc_target_lane_ids_;
  std::vector<std::string> adc_target_lane_seq_;
};

}  // namespace prediction
}  // namespace apollo
