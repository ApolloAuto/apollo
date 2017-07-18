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

#include <utility>

#include "modules/prediction/evaluator/vehicle/vehicle_evaluator.h"

namespace apollo {
namespace prediction {

VehicleEvaluator::VehicleEvaluator() : evaluator_(nullptr) {
  Init();
}

void VehicleEvaluator::Init() {
  CHECK(map_evaluators_.find("DefaultVehicleEvaluator") !=
        map_evaluators_.end());
  evaluator_ = map_evaluators_["DefaultVehicleEvaluator"].get();

  // TODO(kechxu) load user defined model taking over the default one

  CHECK(evaluator_ != nullptr);
}

void VehicleEvaluator::Evaluate(Obstacle* obstacle) {
  evaluator_->Evaluate(obstacle);
}

void VehicleEvaluator::RegisterClass(const std::string& name,
                                     std::unique_ptr<Evaluator> ptr) {
  map_evaluators_[name] = std::move(ptr);
}

}  // namespace prediction
}  // namespace apollo
