/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <memory>
#include <vector>

#include "modules/perception/common/base/hdmap_struct.h"
#include "modules/perception/common/base/lane_struct.h"
#include "modules/perception/common/base/object_pool_types.h"
#include "modules/perception/common/base/traffic_light.h"
#include "modules/perception/common/camera/common/data_provider.h"

namespace apollo {
namespace perception {
namespace camera {

struct CameraFrame {
  // frame sequence id
  std::uint64_t frame_id;
  // timestamp
  double timestamp;
  std::vector<std::shared_ptr<DataProvider>> data_provider;
  std::vector<base::ObjectPtr> detected_objects;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
