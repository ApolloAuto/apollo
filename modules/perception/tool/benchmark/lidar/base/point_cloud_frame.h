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
#pragma once
#include <string>
#include <vector>
#include "modules/perception/tool/benchmark/lidar/util/types.h"

namespace apollo {
namespace perception {
namespace benchmark {

class PointCloudFrame {
 public:
  PointCloudFrame() = default;

  ~PointCloudFrame() = default;

  bool load(const std::vector<std::string>& filenames);

  inline const PointCloudConstPtr get_point_cloud() const {
    return _point_cloud;
  }

  inline std::string get_name() const { return _name; }

  inline void release() {}

  static void set_cloud_type(const std::string& type);

 private:
  PointCloudPtr _point_cloud;
  std::string _name;

 private:
  static std::string _s_cloud_type;
};

}  // namespace benchmark
}  // namespace perception
}  // namespace apollo
