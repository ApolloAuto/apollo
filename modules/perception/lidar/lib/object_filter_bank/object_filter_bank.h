/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#include <vector>
#include <memory>
#include <utility>

#include "cyber/common/macros.h"
#include "modules/perception/lidar/lib/interface/base_object_filter.h"
#include "modules/perception/pipeline/stage.h"

namespace apollo {
namespace perception {
namespace lidar {

class ObjectFilterBank : public pipeline::Stage {
 public:
  using DataFrame = pipeline::DataFrame;
  using Plugin = pipeline::Plugin;
  using PluginType = pipeline::PluginType;
  using StageConfig = pipeline::StageConfig;

 public:
  ObjectFilterBank() = default;

  ~ObjectFilterBank() {
    for (auto& filter : filter_bank_) {
      delete filter;
    }
  }

  bool Init(const ObjectFilterInitOptions& options = ObjectFilterInitOptions());

  // @brief: filter objects
  // @param [in]: options
  // @param [in/out]: frame
  // segmented_objects should be valid, and will be filtered,
  bool Filter(const ObjectFilterOptions& options, LidarFrame* frame);

  size_t Size() const { return filter_bank_.size(); }

  bool Init(const StageConfig& stage_config) override;

  bool Process(DataFrame* data_frame) override;

  bool IsEnabled() const override { return enable_; }

  std::string Name() const override { return name_; }

 private:
  std::vector<BaseObjectFilter*> filter_bank_;

  std::vector<std::unique_ptr<BaseObjectFilter>> filter_ptrs_;
  ObjectFilterBankConfig object_filter_bank_config_;

  DISALLOW_COPY_AND_ASSIGN(ObjectFilterBank);
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
