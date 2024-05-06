/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/camera_tracking/common/similar.h"

#include "modules/perception/common/base/blob.h"

namespace apollo {
namespace perception {
namespace camera {

bool CosineSimilar::Calc(CameraTrackingFrame *frame1,
                         CameraTrackingFrame *frame2, base::Blob<float> *sim) {
  auto n = frame1->detected_objects.size();
  auto m = frame2->detected_objects.size();
  if ((n && m) == 0) {
    return false;
  }
  sim->Reshape({static_cast<int>(n), static_cast<int>(m)});
  float *sim_data = sim->mutable_cpu_data();
  auto dim =
      frame1->detected_objects[0]->camera_supplement.object_feature.size();
  for (auto &object1 : frame1->detected_objects) {
    for (auto &object2 : frame2->detected_objects) {
      float s = 0.0f;
      for (size_t k = 0; k < dim; ++k) {
        s += object1->camera_supplement.object_feature[k] *
             object2->camera_supplement.object_feature[k];
      }
      *sim_data = s;
      ++sim_data;
    }
  }
  return true;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
