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

#include "modules/localization/msf/local_map/base_map/base_map_config.h"
#include "modules/localization/msf/local_map/base_map/base_map_node_index.h"
#include "modules/localization/msf/local_map/base_map/base_map_pool.h"
#include "modules/localization/msf/local_map/lossless_map/lossless_map_matrix.h"
#include "modules/localization/msf/local_map/lossless_map/lossless_map_node.h"

namespace apollo {
namespace localization {
namespace msf {

/**@brief The memory pool for the data structure of BaseMapNode. */
class LosslessMapNodePool : public BaseMapNodePool {
 public:
  /**@brief Constructor
   * @param <pool_size> The memory pool size.
   * @param <thread_size> The thread pool size.
   */
  LosslessMapNodePool(unsigned int pool_size, unsigned int thread_size);
  /**@brief Destructor */
  virtual ~LosslessMapNodePool() = default;

 private:
  virtual BaseMapNode* AllocNewMapNode();
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
