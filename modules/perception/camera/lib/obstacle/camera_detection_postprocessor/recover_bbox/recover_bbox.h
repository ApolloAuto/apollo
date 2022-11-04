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

class RecoverBbox : public pipeline::Plugin {
 public:
  using PluginConfig = pipeline::PluginConfig;
  using DataFrame = pipeline::DataFrame;

 public:
  RecoverBbox() = default;

  explicit RecoverBbox(const PluginConfig& plugin_config);

  virtual ~RecoverBbox() = default;

  bool Init(const PluginConfig &plugin_config) override;

  bool Process(DataFrame *data_frame);

  bool IsEnabled() const override { return enable_; }

  std::string Name() const override { return name_; }

 private:
  void recover_bbox(int roi_w, int roi_h, int offset_y,
                  std::vector<base::ObjectPtr> *objects);
  int roi_w_;
  int roi_h_;
  int offset_y_;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
