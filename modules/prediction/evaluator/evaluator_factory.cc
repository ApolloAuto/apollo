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

#include "modules/prediction/evaluator/evaluator_factory.h"

#include "modules/prediction/evaluator/vehicle/mlp_evaluator.h"
#include "modules/common/log.h"

namespace apollo {
namespace prediction {

EvaluatorFactory::EvaluatorFactory() {
  RegisterEvaluator();
}

void EvaluatorFactory::RegisterEvaluator() {
  Register(ObstacleConf::MLP_EVALUATOR,
        []() -> Evaluator* { return new MLPEvaluator(); });
}

std::unique_ptr<Evaluator> EvaluatorFactory::CreateEvaluator(
    const ObstacleConf::EvaluatorType& type) {
  auto evaluator = CreateObject(type);
  if (!evaluator) {
    AERROR << "Failed to create an evaluator with " << type;
  } else {
    ADEBUG << "Succeeded in creating an evaluator with " << type;
  }
  return evaluator;
}

}  // namespace prediction
}  // namespace apollo
