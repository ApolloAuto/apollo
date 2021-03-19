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

#include "modules/localization/msf/local_map/lossy_map/lossy_map_pool_2d.h"

namespace apollo {
namespace localization {
namespace msf {

LossyMapNodePool2D::LossyMapNodePool2D(unsigned int pool_size,
                                       unsigned int thread_size)
    : BaseMapNodePool(pool_size, thread_size) {}

BaseMapNode* LossyMapNodePool2D::AllocNewMapNode() {
  return new LossyMapNode2D();
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
