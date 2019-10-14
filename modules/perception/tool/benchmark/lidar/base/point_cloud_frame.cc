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

#include "modules/perception/tool/benchmark/lidar/base/point_cloud_frame.h"
#include <vector>
#include "modules/perception/tool/benchmark/lidar/util/io_util.h"

namespace apollo {
namespace perception {
namespace benchmark {

std::string PointCloudFrame::_s_cloud_type = "xyzit";  // NOLINT

void PointCloudFrame::set_cloud_type(const std::string& type) {
  _s_cloud_type = type;
}

bool PointCloudFrame::load(const std::vector<std::string>& filenames) {
  if (filenames.empty()) {
    std::cerr << "Filenames is empty" << std::endl;
    return false;
  }
  _name = filenames[0];
  std::cout << _name << std::endl;
  _point_cloud.reset(new PointCloud);
  if (!load_pcl_pcds(filenames[0], _point_cloud, _s_cloud_type)) {
    std::cerr << "Fail to load pcds: " << filenames[0] << std::endl;
    return false;
  }
  return true;
}

}  // namespace benchmark
}  // namespace perception
}  // namespace apollo
