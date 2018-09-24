/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#ifndef PERCEPTION_LIDAR_LIB_TRACKER_COMMON_TRACK_POOL_TYPES_H_
#define PERCEPTION_LIDAR_LIB_TRACKER_COMMON_TRACK_POOL_TYPES_H_
#include "modules/perception/base/concurrent_object_pool.h"
#include "modules/perception/lidar/lib/tracker/common/track_data.h"
#include "modules/perception/lidar/lib/tracker/common/tracked_object.h"

namespace apollo {
namespace perception {
namespace lidar {

struct TrackedObjectInitializer {
  void operator()(TrackedObject* object) const { object->Reset(); }
};

struct TrackDataInitializer {
  void operator()(TrackData* data) const { data->Reset(); }
};

static const size_t kTrackedObjectPoolSize = 1000;
static const size_t kTrackDataPoolSize = 1000;

typedef base::ConcurrentObjectPool<TrackedObject, kTrackedObjectPoolSize,
                                   TrackedObjectInitializer>
    TrackedObjectPool;

typedef base::ConcurrentObjectPool<TrackData, kTrackDataPoolSize,
                                   TrackDataInitializer>
    TrackDataPool;

}  // namespace lidar
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_LIDAR_LIB_TRACKER_COMMON_TRACK_POOL_TYPES_H_
