/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
#include "modules/prediction/evaluator/vehicle/semantic_lstm_evaluator.h"

/**
 * @namespace apollo::perception
 * @brief apollo::perception
 */
namespace apollo {
namespace perception {

using apollo::prediction::ObstaclesContainer;
using apollo::prediction::Obstacle;
using apollo::prediction::SemanticLSTMEvaluator;
using apollo::prediction::ObstacleHistory;

class EvaluatorManager {
 public:
  EvaluatorManager() = default;
  /**
   * @brief Destructor
   */
  virtual ~EvaluatorManager() = default;

  /* initialize
   */
  void Init();
  /**
   * @brief Run evaluators
   */
  void Run(ObstaclesContainer* obstacles_container);

  void EvaluateObstacle(Obstacle* obstacle,
                        ObstaclesContainer* obstacles_container,
                        std::vector<Obstacle*> dynamic_env);

  void EvaluateObstacle(Obstacle* obstacle,
                        ObstaclesContainer* obstacles_container);

 private:
  void BuildObstacleIdHistoryMap(ObstaclesContainer* obstacles_container);

  std::unordered_map<int, ObstacleHistory> obstacle_id_history_map_;
  std::unique_ptr<SemanticLSTMEvaluator> evaluator_;
};

}  // namespace perception
}  // namespace apollo
