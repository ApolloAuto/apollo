/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
 **/

#pragma once

#include <memory>
#include <vector>

#include "torch/script.h"
#include "torch/torch.h"

#include "modules/common/status/status.h"
#include "modules/common/util/factory.h"
#include "modules/planning/scenarios/scenario.h"
#include "modules/planning/scenarios/stage.h"

namespace apollo {
namespace planning {
namespace scenario {

class TestLearningModelScenario : public Scenario {
 public:
  TestLearningModelScenario(const ScenarioConfig& config,
                            const ScenarioContext* context);

  // TODO(all): continue to refactor scenario framework to
  //            make output more clear
  ScenarioStatus Process(
      const common::TrajectoryPoint& planning_init_point,
      Frame* frame) override;

  std::unique_ptr<Stage> CreateStage(
      const ScenarioConfig::StageConfig& stage_config) override;

 private:
  bool ExtractFeatures(Frame* frame,
                       std::vector<torch::jit::IValue> *input_features);
  bool InferenceModel(const std::vector<torch::jit::IValue> &input_features,
                      Frame* frame);

  torch::jit::script::Module model_;
  torch::Device device_;
  int input_feature_num_ = 0;
  bool is_init_ = false;
};

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
