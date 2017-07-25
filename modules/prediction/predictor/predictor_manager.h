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
 * @brief Use predictor manager to manage all predictors
 */

#ifndef MODULES_PREDICTION_PREDICTOR_PREDICTOR_MANAGER_H_
#define MODULES_PREDICTION_PREDICTOR_PREDICTOR_MANAGER_H_

#include <unordered_map>

#include "modules/prediction/predictor/predictor.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/prediction/proto/prediction_conf.pb.h"
#include "modules/common/macro.h"

/**
 * @namespace apollo::prediction
 * @brief apollo::prediction
 */
namespace apollo {
namespace prediction {

class PredictorManager {
 public:
  /**
   * @brief Destructor
   */ 
  virtual ~PredictorManager() = default;

  /**
   * @brief Get predictor
   * @return Pointer to the predictor
   */
  Predictor* GetPredictor(const ObstacleConf::PredictorType& type);

  /**
   * @brief Execute the predictor generation on perception obstacles
   * @param Perception obstacles
   */
  void Run(
      const ::apollo::perception::PerceptionObstacles& perception_obstacles);

  /**
   * @brief Get prediction obstacles
   * @return Prediction obstacles
   */
  const PredictionObstacles& prediction_obstacles();

 private:
  PredictionObstacles prediction_obstacles_;

  DECLARE_SINGLETON(PredictorManager)
};

}  // namespace prediction
}  // namespace apollo

#endif  // MODULES_PREDICTION_PREDICTOR_PREDICTOR_MANAGER_H_

