/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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

#include <string>
#include <utility>
#include <vector>

#include "torch/extension.h"
#include "torch/script.h"
#include "modules/prediction/evaluator/evaluator.h"
#include "modules/prediction/evaluator/model_manager/model_manager.h"
#include "modules/prediction/pipeline/vector_net.h"
#include "modules/prediction/container/adc_trajectory/adc_trajectory_container.h"

namespace apollo {
namespace prediction {

using apollo::common::TrajectoryPoint;

class MultiAgentEvaluator : public Evaluator {
 public:
  /**
   * @brief Constructor
   */
  MultiAgentEvaluator();

  /**
   * @brief Destructor
   */
  virtual ~MultiAgentEvaluator() = default;

  /**
   * @brief Clear obstacle feature map
   */
  void Clear();

  /**
   * @brief Process obstacle position to vector
   * @param obstacles_container
   * @param adc_trajectory_container
   * @param prediction_obs_ids
   * @param Tensor: all obstacle hisotry pos
   * @param Tensor: all obstacle hisotry pos step
   * @param Tensor: vector mask
   * @param Tensor: all obstacle data
   * @param Tensor: all obstacle p_id
   * @param Tensor: all obstacle position
   */
  bool VectornetProcessObstaclePosition(
    ObstaclesContainer* obstacles_container,
    const ADCTrajectoryContainer* adc_trajectory_container,
    std::vector<int>& prediction_obs_ids,
    torch::Tensor* ptr_multi_obstacle_pos,
    torch::Tensor* ptr_multi_obstacle_pos_step,
    torch::Tensor* ptr_vector_mask,
    torch::Tensor* ptr_all_obs_data,
    torch::Tensor* ptr_all_obs_p_id,
    torch::Tensor* ptr_multi_obstacle_position);

  /**
  * @brief Process map data to vector
  * @param FeatureVector: map feature vector
  * @param PidVector: map p_id vector
  * @param int: obstacle number
  * @param Tensor: map data
  * @param Tensor: map data p_id
  * @param Tensor: vector mask
  */
  bool VectornetProcessMapData(FeatureVector *map_feature,
                               PidVector *map_p_id,
                               const int obs_num,
                               torch::Tensor* ptr_map_data,
                               torch::Tensor* ptr_all_map_p_id,
                               torch::Tensor* ptr_vector_mask);

  /**
   * @brief Override Evaluate
   * @param obstacle
   * @param obstacles_container
   */
  bool Evaluate(Obstacle* obstacle, ObstaclesContainer* obstacles_container) override;

  /**
   * @brief Override Evaluate
   * @param adc_trajectory_container
   * @param obstacle
   * @param obstacles_container
   */
  bool Evaluate(const ADCTrajectoryContainer* adc_trajectory_container,
                Obstacle* obstacle,
                ObstaclesContainer* obstacles_container) override;

  /**
   * @brief Extract all obstacles history
   * @param obstacles_container
   *        Feature container in a vector for receiving the obstacle history
   * @param prediction_obs_ids: obstacle ids
   * @param trajectory_points: trajectory points
   * @param multi_obs_pos: multi obstacle hisotry pos
   * @param multi_obs_position: multi obstacle position
   * @param all_obs_length: alll obstacle length
   * @param all_obs_pos_history: all obstacle history pos
   * @param adc_traj_curr_pos: adc trajectory current pos
   * @param vector_mask: vector mask
   */
  bool ExtractObstaclesHistory(
    ObstaclesContainer* obstacles_container,
    std::vector<int>& prediction_obs_ids,
    std::vector<TrajectoryPoint>* trajectory_points,
    std::vector<std::vector<std::pair<double, double>>>* multi_obs_pos,
    std::vector<std::vector<double>>* multi_obs_position,
    std::vector<std::pair<double, double>>* all_obs_length,
    std::vector<std::vector<std::pair<double, double>>>* all_obs_pos_history,
    std::vector<std::pair<double, double>>* adc_traj_curr_pos,
    torch::Tensor* vector_mask);

  /**
   * @brief Get the name of evaluator.
   */
  std::string GetName() override { return "MULTI_AGENT_EVALUATOR"; }

 private:
  /**
   * @brief Load model file
   */
  void LoadModel();

 private:
  int max_agent_num = 50;
  int obs_num = 0;
  int vector_obs_num = 0;
  bool with_planning_traj = true;
  ModelManager model_manager_;
  at::Tensor torch_default_output_tensor_;
  Model::Backend device_;
  VectorNet vector_net_;
};

}  // namespace prediction
}  // namespace apollo
