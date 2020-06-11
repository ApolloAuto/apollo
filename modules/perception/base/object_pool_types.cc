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
#include "modules/perception/base/object_pool_types.h"

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace base {

// @brief call pool instance once to initialize memory
__attribute__((constructor)) void PoolInitialize() {
  ObjectPool::Instance();
  PointFCloudPool::Instance();
  PointDCloudPool::Instance();
  FramePool::Instance();
#ifndef PERCEPTION_BASE_DISABLE_POOL
  AINFO << "Initialize base object pool (no-malloc).";
#else
  AINFO << "Initialize base object pool (malloc).";
#endif
}

}  // namespace base
}  // namespace perception
}  // namespace apollo
