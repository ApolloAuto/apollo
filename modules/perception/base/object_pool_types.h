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
#ifndef PERCEPTION_BASE_OBJECT_POOL_TYPES_H_
#define PERCEPTION_BASE_OBJECT_POOL_TYPES_H_
#include "modules/perception/base/concurrent_object_pool.h"
#include "modules/perception/base/frame.h"
#include "modules/perception/base/object.h"
#include "modules/perception/base/point_cloud_types.h"

namespace apollo {
namespace perception {
namespace base {

struct ObjectInitializer {
  void operator()(Object* object) const { object->Reset(); }
};

template <typename T>
struct PointCloudInitializer {
  void operator()(AttributePointCloud<Point<T>>* cloud) const {
    cloud->clear();
  }
};

struct FrameInitializer {
  void operator()(Frame* frame) const { frame->Reset(); }
};

static const size_t kObjectPoolSize = 10000;
static const size_t kPointCloudPoolSize = 1000;
static const size_t kFramePoolSize = 100;

// pool typedef collections,
// developer should add pool type to the PoolInitialize function in .cc file
typedef ConcurrentObjectPool<Object, kObjectPoolSize, ObjectInitializer>
    ObjectPool;

typedef ConcurrentObjectPool<PointFCloud, kPointCloudPoolSize,
                             PointCloudInitializer<float>>
    PointFCloudPool;

typedef ConcurrentObjectPool<PointDCloud, kPointCloudPoolSize,
                             PointCloudInitializer<double>>
    PointDCloudPool;

typedef ConcurrentObjectPool<Frame, kFramePoolSize, FrameInitializer> FramePool;

// dummy version, still allocating memory each time
// typedef DummyObjectPool<Object> ObjectPool;

}  // namespace base
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_BASE_OBJECT_POOL_TYPES_H_
