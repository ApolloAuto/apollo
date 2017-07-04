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

#include "modules/prediction/prediction.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/time/time.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"

namespace apollo {
namespace prediction {

using ::apollo::perception::PerceptionObstacles;
using ::apollo::common::adapter::AdapterManager;
using ::apollo::common::time::Clock;

std::string Prediction::Name() const {
  return FLAGS_prediction_module_name;
}

apollo::common::Status Prediction::Init() {
  AdapterManager::instance()->Init();
  AdapterManager::SetPerceptionObstaclesCallback(&Prediction::OnPerception,
                                                 this);
  return apollo::common::Status::OK();
}

apollo::common::Status Prediction::Start() {
  return apollo::common::Status::OK();
}

void Prediction::Stop() {}

void Prediction::OnPerception(const PerceptionObstacles &perception_obstacles) {
  PredictionObstacles prediction_obstacles;
  AdapterManager::FillPredictionHeader(Name(),
                                       prediction_obstacles.mutable_header());
  AdapterManager::PublishPrediction(prediction_obstacles);

  ADEBUG << prediction_obstacles.ShortDebugString();
}

}  // namespace prediction
}  // namespace apollo
