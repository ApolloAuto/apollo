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

#include "modules/prediction/predictor/vehicle/lane_sequence_predictor.h"
#include "modules/prediction/predictor/vehicle/free_move_predictor.h"

#include "modules/common/log.h"

namespace apollo {
namespace prediction {

PredictorFactory::PredictorFactory() {
  RegisterPredictor();
}

void PredictorFactory::RegisterPredictor() {
    Register(ObstacleConf::LANE_SEQUENCE_PREDICTOR,
        []() -> Predictor* { return new LaneSequencePredictor(); });
    Register(ObstacleConf::FREE_MOVE_PREDICTOR,
        []() -> Predictor* { return new FreeMovePredictor(); });
}

std::unique_ptr<Predictor> PredictorFactory::CreatePredictor(
    const ObstacleConf::PredictorType& type) {
  auto predictor = CreateObject(type);
  if (!predictor) {
    AERROR << "Failed to create a predictor with " << type;
  } else {
    ADEBUG << "Succeeded in creating a predictor with " << type;
  }
  return predictor;
}

}  // namespace prediction
}  // namespace apollo
