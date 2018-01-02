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

#ifndef MODULES_PREDICTION_CONTAINER_ADC_TRAJECTORY_OBSTACLES_H_
#define MODULES_PREDICTION_CONTAINER_ADC_TRAJECTORY_OBSTACLES_H_

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "Eigen/Dense"

#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/polygon2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/prediction/container/container.h"
#include "modules/prediction/proto/lane_graph.pb.h"

namespace apollo {
namespace prediction {

class ADCTrajectoryContainer : public Container {
 public:
  /**
   * @brief Constructor
   */
  ADCTrajectoryContainer() = default;

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
  bool IsPointInJunction(const apollo::common::PathPoint& point) const;

  /**
   * @brief Has overlap with ADC trajectory
   * @return True if a target lane sequence has overlap with ADC trajectory
   */
  bool HasOverlap(const LaneSequence& lane_sequence);

  /**
   * @brief Set ADC position
   */
  void SetPosition(const ::apollo::common::math::Vec2d& position);

 private:
  void SetJunctionPolygon();

  void SetLaneSequence();

  std::string ToString(const std::unordered_set<std::string>& lane_ids);

  std::string ToString(const std::vector<std::string>& lane_ids);

 private:
  ::apollo::planning::ADCTrajectory adc_trajectory_;
  ::apollo::common::math::Polygon2d adc_junction_polygon_;
  std::unordered_set<std::string> adc_lane_ids_;
  std::vector<std::string> adc_lane_seq_;
};

}  // namespace prediction
}  // namespace apollo

#endif  // MODULES_PREDICTION_CONTAINER_ADC_TRAJECTORY_OBSTACLES_H_
