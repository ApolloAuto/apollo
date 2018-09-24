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
#ifndef PERCEPTION_LIDAR_LIB_COMMON_LIDAR_FRAME_POOL_H_
#define PERCEPTION_LIDAR_LIB_COMMON_LIDAR_FRAME_POOL_H_
#include "modules/perception/base/concurrent_object_pool.h"
#include "modules/perception/lidar/common/lidar_frame.h"

namespace apollo {
namespace perception {
namespace lidar {

struct LidarFrameInitializer {
  void operator()(LidarFrame* frame) const { frame->Reset(); }
};

static const size_t kLidarFramePoolSize = 50;

typedef base::ConcurrentObjectPool<LidarFrame, kLidarFramePoolSize,
                                   LidarFrameInitializer>
    LidarFramePool;

}  // namespace lidar
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_LIDAR_LIB_COMMON_LIDAR_FRAME_POOL_H_
