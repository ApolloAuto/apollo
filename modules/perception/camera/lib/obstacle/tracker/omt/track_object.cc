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
#include "modules/perception/camera/lib/obstacle/tracker/omt/track_object.h"

namespace apollo {
namespace perception {
namespace camera {

// void TrackObjectPool::Init(size_t capacity) {
//   for (size_t i = 0; i < capacity; ++i) {
//     data_.push(std::make_shared<TrackObject>());
//   }
//   capacity_ = capacity;
// }
// void TrackObjectPool::NewObject(TrackObjectPtr *ptr) {
//   if (data_.empty()) {
//     for (size_t i = 0; i < capacity_; ++i) {
//       data_.push(std::make_shared<TrackObject>());
//     }
//     capacity_ *= 2;
//   }
//   *ptr = data_.front();
//   data_.pop();
// }
// void TrackObjectPool::NewObject(TrackObjectPtrs *ptr, int batch_size) {
//   ptr->resize(batch_size);
//   for (int i = 0; i < batch_size; ++i) {
//     NewObject(&(ptr->at(i)));
//   }
// }
// void TrackObjectPool::FreeObject(TrackObjectPtr ptr) {
//   data_.push(ptr);
// }
// void TrackObjectPool::FreeObject(const TrackObjectPtrs &ptr) {
//   for (const auto &i : ptr) {
//     data_.push(i);
//   }
// }

}  // namespace camera
}  // namespace perception
}  // namespace apollo
