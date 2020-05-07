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

#include <vector>

#include "torch/script.h"
#include "torch/torch.h"

#include "modules/planning/scenarios/learning_model/learning_model_sample_scenario.h"
#include "modules/planning/scenarios/stage.h"

namespace apollo {
namespace planning {
namespace scenario {

struct LearningModelSampleContext;

class LearningModelSampleStageRun : public Stage {
 public:
  explicit LearningModelSampleStageRun(
      const ScenarioConfig::StageConfig& config)
      : Stage(config), device_(torch::kCPU) {}

 private:
  Stage::StageStatus Process(const common::TrajectoryPoint& planning_init_point,
                             Frame* frame) override;
  LearningModelSampleContext* GetContext() {
    return GetContextAs<LearningModelSampleContext>();
  }
  bool ExtractFeatures(Frame* frame,
                       std::vector<torch::jit::IValue> *input_features);
  bool InferenceModel(const std::vector<torch::jit::IValue> &input_features,
                      Frame* frame);
  Stage::StageStatus FinishStage();

 private:
  ScenarioLearningModelSampleConfig scenario_config_;
  torch::Device device_;
  torch::jit::script::Module model_;
  int input_feature_num_ = 0;
};

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
