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

#include "modules/perception/base/blob.h"
#include "modules/perception/camera/lib/obstacle/tracker/omt/frame_list.h"

namespace apollo {
namespace perception {
namespace camera {
struct TrackObject {
  PatchIndicator indicator;
  double timestamp;
  base::BBox2DF projected_box;
  base::ObjectPtr object;
};
typedef std::shared_ptr<TrackObject> TrackObjectPtr;
typedef std::vector<TrackObjectPtr> TrackObjectPtrs;
// struct TrackFeature {
//   TrackFeature(int frame_id, std::shared_ptr<base::Blob<float>> blob);
//   int frame_id_;
//   std::shared_ptr<base::Blob<float>> blob_;
// };
// class TrackObjectPool {
//  public:
//   void Init(size_t capacity);
//   void NewObject(TrackObjectPtr *ptr);
//   void NewObject(TrackObjectPtrs *ptr, int batch_size);
//   void FreeObject(TrackObjectPtr ptr);
//   void FreeObject(const TrackObjectPtrs &ptr);
//  private:
//   std::queue<TrackObjectPtr> data_;
//   size_t capacity_;
// };

}  // namespace camera
}  // namespace perception
}  // namespace apollo
