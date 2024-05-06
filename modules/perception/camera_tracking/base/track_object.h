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

#include "modules/perception/camera_tracking/base/frame_list.h"
#include "modules/perception/common/base/object.h"

namespace apollo {
namespace perception {
namespace camera {

struct alignas(16) TrackObject {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PatchIndicator indicator;
  double timestamp;
  base::BBox2DF projected_box;
  base::ObjectPtr object;
};

typedef std::shared_ptr<TrackObject> TrackObjectPtr;
typedef std::vector<TrackObjectPtr> TrackObjectPtrs;

}  // namespace camera
}  // namespace perception
}  // namespace apollo
