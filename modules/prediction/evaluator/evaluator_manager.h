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
 * @brief Use evaluator manager to manage all evaluators
 */

#pragma once

#include <list>
#include <map>
#include <memory>
#include <unordered_map>
#include <vector>

#include "cyber/common/macros.h"
#include "modules/prediction/common/semantic_map.h"
#include "modules/prediction/evaluator/evaluator.h"
#include "modules/prediction/proto/prediction_conf.pb.h"
#include "modules/prediction/pipeline/vector_net.h"

/**
 * @namespace apollo::prediction
 * @brief apollo::prediction
 */
namespace apollo {
namespace prediction {

class EvaluatorManager {
 public:
  /**
   * @brief Constructor
   */
  EvaluatorManager();

  /**
   * @brief Destructor
   */
  virtual ~EvaluatorManager() = default;

  /**
   * @brief Initializer
   * @param Prediction config
   */
  void Init(const PredictionConf& config);

  /**
   * @brief Get evaluator
   * @return Pointer to the evaluator
   */
  Evaluator* GetEvaluator(const ObstacleConf::EvaluatorType& type);

  /**
   * @brief Run evaluators
   */
  void Run(const ADCTrajectoryContainer* adc_trajectory_container,
           ObstaclesContainer* obstacles_container);

  void EvaluateObstacle(const ADCTrajectoryContainer* adc_trajectory_container,
                        Obstacle* obstacle,
                        ObstaclesContainer* obstacles_container,
                        std::vector<Obstacle*> dynamic_env);

  void EvaluateObstacle(Obstacle* obstacle,
                        ObstaclesContainer* obstacles_container);

  void EvaluateMultiObstacle(
    const ADCTrajectoryContainer* adc_trajectory_container,
    ObstaclesContainer* obstacles_container);

 private:
  void BuildObstacleIdHistoryMap(ObstaclesContainer* obstacles_container,
                                 size_t max_num_frame);

  void DumpCurrentFrameEnv(ObstaclesContainer* obstacles_container);

  /**
   * @brief Register an evaluator by type
   * @param Evaluator type
   */
  void RegisterEvaluator(const ObstacleConf::EvaluatorType& type);

  /**
   * @brief Create an evaluator by type
   * @param Evaluator type
   * @return A unique pointer to the evaluator
   */
  std::unique_ptr<Evaluator> CreateEvaluator(
      const ObstacleConf::EvaluatorType& type);

  /**
   * @brief Register all evaluators
   */
  void RegisterEvaluators();

 private:
  std::map<ObstacleConf::EvaluatorType, std::unique_ptr<Evaluator>> evaluators_;

  ObstacleConf::EvaluatorType vehicle_on_lane_evaluator_ =
      ObstacleConf::CRUISE_MLP_EVALUATOR;

  ObstacleConf::EvaluatorType vehicle_on_lane_caution_evaluator_ =
      ObstacleConf::CRUISE_MLP_EVALUATOR;

  ObstacleConf::EvaluatorType vehicle_in_junction_evaluator_ =
      ObstacleConf::JUNCTION_MLP_EVALUATOR;

  ObstacleConf::EvaluatorType vehicle_in_junction_caution_evaluator_ =
      ObstacleConf::JUNCTION_MAP_EVALUATOR;

  ObstacleConf::EvaluatorType vehicle_default_caution_evaluator_ =
      ObstacleConf::SEMANTIC_LSTM_EVALUATOR;

  ObstacleConf::EvaluatorType cyclist_on_lane_evaluator_ =
      ObstacleConf::CYCLIST_KEEP_LANE_EVALUATOR;

  ObstacleConf::EvaluatorType pedestrian_evaluator_ =
      ObstacleConf::SEMANTIC_LSTM_EVALUATOR;

  ObstacleConf::EvaluatorType vectornet_evaluator_ =
      ObstacleConf::VECTORNET_EVALUATOR;

  ObstacleConf::EvaluatorType default_on_lane_evaluator_ =
      ObstacleConf::MLP_EVALUATOR;

  ObstacleConf::EvaluatorType interaction_evaluator_ =
      ObstacleConf::JOINTLY_PREDICTION_PLANNING_EVALUATOR;

  ObstacleConf::EvaluatorType multi_agent_evaluator_ =
      ObstacleConf::MULTI_AGENT_EVALUATOR;

  std::unordered_map<int, ObstacleHistory> obstacle_id_history_map_;

  std::unique_ptr<SemanticMap> semantic_map_;
};

}  // namespace prediction
}  // namespace apollo
