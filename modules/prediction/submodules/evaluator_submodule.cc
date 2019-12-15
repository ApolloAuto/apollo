/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/prediction/submodules/evaluator_submodule.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/adapters/proto/adapter_config.pb.h"
#include "modules/common/time/time.h"
#include "modules/prediction/common/message_process.h"
#include "modules/prediction/common/prediction_system_gflags.h"
#include "modules/prediction/evaluator/evaluator_manager.h"

namespace apollo {
namespace prediction {

EvaluatorSubmodule::~EvaluatorSubmodule() {}

std::string EvaluatorSubmodule::Name() const {
  return FLAGS_evaluator_submodule_name;
}

bool EvaluatorSubmodule::Init() {
  if (!MessageProcess::InitEvaluators()) {
    return false;
  }
  // TODO(kechxu) change topic name when finalized
  evaluator_writer_ =
      node_->CreateWriter<SubmoduleOutput>(FLAGS_evaluator_topic_name);
  return true;
}

bool EvaluatorSubmodule::Proc(
    const std::shared_ptr<SubmoduleOutput>& container_output) {
  constexpr static size_t kHistorySize = 1;
  const auto frame_start_time = container_output->frame_start_time();
  ObstaclesContainer obstacles_container(*container_output);
  EvaluatorManager::Instance()->Run(&obstacles_container);
  SubmoduleOutput submodule_output =
      obstacles_container.GetSubmoduleOutput(kHistorySize, frame_start_time);
  evaluator_writer_->Write(submodule_output);
  return true;
}

}  // namespace prediction
}  // namespace apollo
