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

#ifndef MODULES_PREDICTION_EVALUATOR_EVALUATOR_MANAGER_H_
#define MODULES_PREDICTION_EVALUATOR_EVALUATOR_MANAGER_H_

#include <map>
#include <memory>

#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/prediction/proto/prediction_conf.pb.h"

#include "modules/common/macro.h"
#include "modules/prediction/evaluator/evaluator.h"

/**
 * @namespace apollo::prediction
 * @brief apollo::prediction
 */
namespace apollo {
namespace prediction {

class EvaluatorManager {
 public:
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
   * @param Perception obstacles
   */
  void Run(const perception::PerceptionObstacles& perception_obstacles);

 private:
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
      ObstacleConf::MLP_EVALUATOR;

  ObstacleConf::EvaluatorType cyclist_on_lane_evaluator_ =
      ObstacleConf::MLP_EVALUATOR;

  ObstacleConf::EvaluatorType default_on_lane_evaluator_ =
      ObstacleConf::MLP_EVALUATOR;

  DECLARE_SINGLETON(EvaluatorManager)
};

}  // namespace prediction
}  // namespace apollo

#endif  // MODULES_PREDICTION_EVALUATOR_EVALUATOR_MANAGER_H_
