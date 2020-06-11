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

#include "modules/perception/base/light_object_pool.h"
#include "modules/perception/base/object_pool_types.h"
#include "modules/perception/lidar/lib/segmentation/cnnseg/spp_engine/spp_cluster.h"

namespace apollo {
namespace perception {
namespace lidar {

struct SppClusterInitializer {
  void operator()(SppCluster* cluster) const { cluster->clear(); }
};

static const size_t kSppClusterPoolSize = 1000;

typedef base::LightObjectPool<SppCluster, kSppClusterPoolSize,
                              SppClusterInitializer,
                              base::SensorType::VELODYNE_64>
    SppClusterPool;

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
