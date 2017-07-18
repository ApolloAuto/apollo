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
 * @brief Define the vehicle evaluator.
 */

#ifndef MODULES_PREDICTION_EVALUATOR_VEHICLE_VEHICLE_EVALUATOR_H_
#define MODULES_PREDICTION_EVALUATOR_VEHICLE_VEHICLE_EVALUATOR_H_

#include <string>
#include <memory>
#include <unordered_map>

#include "modules/prediction/evaluator/evaluator.h"
#include "modules/prediction/container/obstacles/obstacle.h"

namespace apollo {
namespace prediction {

class VehicleEvaluator : public Evaluator {
 public:
  VehicleEvaluator();

  virtual ~VehicleEvaluator();

  void Evaluate(Obstacle* obstacle);

  void Clear();

 private:
  void RegisterClass(const std::string& name, std::unique_ptr<Evaluator> ptr);

  void Init();

 private:
  Evaluator* evaluator_;
  std::unordered_map<std::string, std::unique_ptr<Evaluator>> map_evaluators_;
};

}  // namespace prediction
}  // namespace apollo

#endif  // MODULES_PREDICTION_EVALUATOR_VEHICLE_VEHICLE_EVALUATOR_H_
