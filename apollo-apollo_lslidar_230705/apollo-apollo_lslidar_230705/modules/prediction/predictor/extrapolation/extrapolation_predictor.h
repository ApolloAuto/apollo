/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
 * @brief Define extrapolation predictor
 */

#pragma once

#include <string>

#include "modules/prediction/container/obstacles/obstacle_clusters.h"
#include "modules/prediction/predictor/sequence/sequence_predictor.h"

namespace apollo {
namespace prediction {

class ExtrapolationPredictor : public SequencePredictor {
 public:
  /**
   * @brief Constructor
   */
  ExtrapolationPredictor();

  /**
   * @brief Destructor
   */
  virtual ~ExtrapolationPredictor() = default;

  /**
   * @brief Make prediction
   * @param ADC trajectory container
   * @param Obstacle pointer
   * @param Obstacles container
   * @return If predicted successfully
   */
  bool Predict(const ADCTrajectoryContainer* adc_trajectory_container,
               Obstacle* obstacle,
               ObstaclesContainer* obstacles_container) override;

 private:
  struct LaneSearchResult {
    bool found = false;
    std::string lane_id = "";
    int point_index = -1;
  };

  void PostProcess(Trajectory* trajectory_ptr, ObstacleClusters* clusters_ptr);

  LaneSearchResult SearchExtrapolationLane(const Trajectory& trajectory,
                                           const int num_tail_point);

  void ExtrapolateByLane(const LaneSearchResult& lane_search_result,
                         const double extrapolation_speed,
                         Trajectory* trajectory_ptr,
                         ObstacleClusters* clusters_ptr);

  void ExtrapolateByFreeMove(const int num_tail_point,
                             const double extrapolation_speed,
                             Trajectory* trajectory_ptr);

  double ComputeExtraplationSpeed(const int num_tail_point,
                                  const Trajectory& trajectory);
};

}  // namespace prediction
}  // namespace apollo
