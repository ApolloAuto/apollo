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

#include "modules/common/log.h"

namespace apollo {
namespace prediction {

EvaluatorFactory::EvaluatorFactory() {}

void EvaluatorFactory::RegisterEvaluator() {
    Register(PredictionConf::DEF_EVAL,
        []() -> Evaluator* { return nullptr; });
}

std::unique_ptr<Evaluator> EvaluatorFactory::CreateEvaluator(
    const PredictionConf::Eval& eval) {
  auto evaluator = CreateObject(eval);
  if (!evaluator) {
    AERROR << "Failed to create an evaluator with " << eval;
  } else {
    ADEBUG << "Succeeded in creating an evaluator with " << eval;
  }
  return evaluator;
}

}
}