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

#include <string>
#include <unordered_map>
#include <vector>

#include "modules/perception/pipeline/proto/pipeline_config.pb.h"

#include "modules/perception/pipeline/data_frame.h"
#include "modules/perception/pipeline/plugin.h"

namespace apollo {
namespace perception {
namespace pipeline {

class Stage {
 public:
  Stage() = default;
  virtual ~Stage() = default;

  virtual bool Init(const StageConfig& stage_config) = 0;

  virtual bool Process(DataFrame* data_frame) = 0;

  virtual bool IsEnabled() const = 0;

  virtual std::string Name() const = 0;

  StageConfig stage_config_;

 protected:
  bool Initialize(const StageConfig& stage_config);

 private:
  void Clear();

 protected:
  bool enable_ = false;
  std::string name_;

  std::unordered_map<PluginType, PluginConfig, std::hash<int>>
      plugin_config_map_;
};

}  // namespace pipeline
}  // namespace perception
}  // namespace apollo
