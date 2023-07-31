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
#pragma once

#include "modules/perception/common/base/concurrent_object_pool.h"
#include "modules/perception/lidar_tracking/tracker/common/mlf_track_data.h"
#include "modules/perception/lidar_tracking/tracker/common/track_data.h"
#include "modules/perception/lidar_tracking/tracker/common/tracked_object.h"

namespace apollo {
namespace perception {
namespace lidar {

struct TrackedObjectInitializer {
  /**
   * @brief Reset tracked object
   *
   * @param object
   */
  void operator()(TrackedObject* object) const { object->Reset(); }
};

struct TrackDataInitializer {
  /**
   * @brief Reset tracked object
   *
   * @param object
   */
  void operator()(TrackData* data) const { data->Reset(); }
};

struct MlfTrackDataInitializer {
  /**
   * @brief Reset tracked object
   *
   * @param object
   */
  void operator()(MlfTrackData* data) const { data->Reset(); }
};

static const size_t kTrackedObjectPoolSize = 20000;
static const size_t kTrackDataPoolSize = 1000;

typedef base::ConcurrentObjectPool<TrackedObject, kTrackedObjectPoolSize,
                                   TrackedObjectInitializer>
    TrackedObjectPool;

typedef base::ConcurrentObjectPool<TrackData, kTrackDataPoolSize,
                                   TrackDataInitializer>
    TrackDataPool;
typedef base::ConcurrentObjectPool<MlfTrackData, kTrackDataPoolSize,
                                   MlfTrackDataInitializer>
    MlfTrackDataPool;

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
