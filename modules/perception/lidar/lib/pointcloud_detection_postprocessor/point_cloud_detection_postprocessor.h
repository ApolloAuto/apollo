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

#include "modules/perception/pipeline/data_frame.h"
#include "modules/perception/pipeline/plugin.h"

namespace apollo {
namespace perception {
namespace lidar {

class PointCloudDetectionPostprocessor : public Stage {
 public:
  PointCloudDetectionPostprocessor() {name_ = "PointCloudDetectionPostprocessor"};

  virtual ~PointCloudDetectionPostprocessor() = default;

  bool Init(const StageConfig& stage_config) override;

  bool Process(DataFrame* data_frame) override;

  bool Process(DataFrame* data_frame);

  bool IsEnabled() override { return enable_; }

  const std::string& Name() const override { return name_; }

 protected:
  bool enable_;


 private:
  std::string name_;
  std::map<TaskConfig::TaskType, std::unique_ptr<Task>> tasks_;
  std::vector<std::unique_ptr<Task>> task_list_;
   std::unique_ptr<Plugin> get_objects_;
};  // class PointCloudDetectionPostprocessor

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
