/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <utility>

#include "modules/perception/pipeline/proto/pipeline_config.pb.h"

#include "modules/perception/pipeline/stage.h"

namespace apollo {
namespace perception {
namespace pipeline {

class Pipeline {
 public:
  Pipeline() = default;
  virtual ~Pipeline() = default;

  virtual bool Init(const PipelineConfig& pipeline_config) = 0;

  virtual bool Process(DataFrame* data_frame) = 0;

  virtual std::string Name() const = 0;

 protected:
  bool Initialize(const PipelineConfig& pipeline_config);
  bool InnerProcess(DataFrame* data_frame);

 private:
  void Clear();

  std::shared_ptr<Stage> CreateStage(const StageType& stage_type);

  bool CheckRepeatedStage(const std::string& stage_name);

 protected:
  std::string name_;

  PipelineConfig pipeline_config_;

  std::unordered_map<StageType, StageConfig, std::hash<int>> stage_config_map_;

  std::vector<std::shared_ptr<Stage>> stage_ptrs_;
};

}  // namespace pipeline
}  // namespace perception
}  // namespace apollo
