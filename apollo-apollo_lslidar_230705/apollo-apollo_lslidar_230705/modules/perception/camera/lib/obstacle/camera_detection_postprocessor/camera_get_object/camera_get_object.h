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
#include <string>

#include "modules/perception/base/box.h"
#include "modules/perception/base/object.h"
#include "modules/perception/pipeline/data_frame.h"
#include "modules/perception/pipeline/plugin.h"

namespace apollo {
namespace perception {
namespace camera {

class CameraGetObject : public pipeline::Plugin {
 public:
  using PluginConfig = pipeline::PluginConfig;
  using DataFrame = pipeline::DataFrame;

 public:
  CameraGetObject() = default;
  explicit CameraGetObject(const PluginConfig& plugin_config);

  virtual ~CameraGetObject() = default;

  bool Init(const PluginConfig& plugin_config) override;

  bool Process(const std::vector<float> &detect_result, DataFrame *data_frame);

  bool IsEnabled() const override { return enable_; }

  std::string Name() const override { return name_; }

 private:
  void get_smoke_objects_cpu(const std::vector<float> &detect_result,
                             float confidence_threshold, int width, int height,
                             std::vector<base::ObjectPtr> *objects);
  void fill_smoke_base(base::ObjectPtr obj, const float *bbox,
                                int width, int height);
  void fill_smoke_bbox3d(bool with_box3d, base::ObjectPtr obj,
                                  const float *bbox);

  float confidence_threshold_;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
