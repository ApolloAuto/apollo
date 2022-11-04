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

#include <vector>
#include <memory>
#include <string>

#include "Eigen/Core"

#include "modules/perception/base/object.h"
#include "modules/perception/pipeline/data_frame.h"
#include "modules/perception/pipeline/plugin.h"

namespace apollo {
namespace perception {
namespace lidar {

class PointCloudGetObjects : public pipeline::Plugin {
 public:
  using DataFrame = pipeline::DataFrame;
  using PluginConfig = pipeline::PluginConfig;

  PointCloudGetObjects() = default;
  explicit PointCloudGetObjects(const PluginConfig& plugin_config);

  virtual ~PointCloudGetObjects() = default;

  bool Init(const PluginConfig& plugin_config) override;

  bool Process(const std::vector<float>& detections,
               const std::vector<int>& labels,
               DataFrame* data_frame);

  bool IsEnabled() const override { return enable_; }
  std::string Name() const override { return name_; }

 private:
  base::ObjectSubType GetObjectsubType(const int label);
  /*
  void PointCloudGetObjects::PointCloudGetObjects(
  std::vector<std::shared_ptr<Object>>* objects, const Eigen::Affine3d& pose,
  std::vector<float>* detections, std::vector<int>* labels);
  */
  void GetObjects(const Eigen::Affine3d& pose,
                  const std::vector<float>& detections,
                  const std::vector<int>& labels,
                  std::vector<std::shared_ptr<base::Object>>* objects);
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
