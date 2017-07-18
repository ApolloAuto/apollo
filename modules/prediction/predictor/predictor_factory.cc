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

#include "modules/prediction/predictor/predictor_factory.h"

#include "modules/common/log.h"

namespace apollo {
namespace prediction {

PredictorFactory::PredictorFactory() {}

void PredictorFactory::RegisterPredictor() {
    Register(ObstacleConf::DEF_VEHICLE_PRED,
        []() -> Predictor* { return nullptr; });
}

std::unique_ptr<Predictor> PredictorFactory::CreatePredictor(
    const ObstacleConf::Pred& pred) {
  auto predictor = CreateObject(pred);
  if (!predictor) {
    AERROR << "Failed to create a predictor with " << pred;
  } else {
    ADEBUG << "Succeeded in creating a predictor with " << pred;
  }
  return predictor;
}

}
}