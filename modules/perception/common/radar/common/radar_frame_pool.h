/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/common/radar/common/radar_frame.h"

namespace apollo {
namespace perception {
namespace radar4d {

struct RadarFrameInitializer {
  void operator()(RadarFrame* frame) const { frame->Reset(); }
};

static const size_t kRadarFramePoolSize = 50;

typedef base::ConcurrentObjectPool<RadarFrame, kRadarFramePoolSize,
                                   RadarFrameInitializer>
    RadarFramePool;

}  // namespace radar4d
}  // namespace perception
}  // namespace apollo
