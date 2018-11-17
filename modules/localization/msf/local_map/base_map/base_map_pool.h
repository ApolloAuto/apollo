/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_LOCALIZATION_MSF_LOCAL_MAP_BASE_MAP_BASE_MAP_POOL_H
#define MODULES_LOCALIZATION_MSF_LOCAL_MAP_BASE_MAP_BASE_MAP_POOL_H

#include <list>
#include <set>

#include "boost/thread.hpp"

#include "modules/localization/msf/common/util/threadpool.h"
#include "modules/localization/msf/local_map/base_map/base_map_fwd.h"

namespace apollo {
namespace localization {
namespace msf {

/**@brief The memory pool for the data structure of BaseMapNode. */
class BaseMapNodePool {
 public:
  /**@brief Constructor
   * @param <pool_size> The memory pool size.
   * @param <thread_size> The thread pool size.
   */
  BaseMapNodePool(unsigned int pool_size, unsigned int thread_size);
  /**@brief Destructor */
  virtual ~BaseMapNodePool();
  /**@brief Initialize the pool.
   * @param <map_config> The map option.
   * @param <is_fixed_size> The flag of pool auto expand.
   * */
  void Initial(const BaseMapConfig* map_config, bool is_fixed_size = true);
  /**@brief Release the pool. */
  void Release();
  /**@brief Get a MapNode object from memory pool.
   * @param <return> The MapNode object.
   * */
  BaseMapNode* AllocMapNode();
  /**@brief Release MapNode object to memory pool.
   * @param <map_node> The released MapNode object.
   * */
  void FreeMapNode(BaseMapNode* map_node);
  /**@brief Get the size of pool. */
  unsigned int GetPoolSize() { return pool_size_; }

 private:
  /**@brief The task function of the thread pool for release node.
   * @param <map_node> The released MapNode object.
   * */
  void FreeMapNodeTask(BaseMapNode* map_node);
  /**@brief new a map node. */
  virtual BaseMapNode* AllocNewMapNode() = 0;
  /**@brief init a map node. */
  void InitNewMapNode(BaseMapNode* node);
  /**@brief Finalize a map node, before reset or delloc the map node. */
  void FinalizeMapNode(BaseMapNode* node);
  /**@brief delloc a map node. */
  void DellocMapNode(BaseMapNode* node);
  /**@brief reset a map node. */
  void ResetMapNode(BaseMapNode* node);

 protected:
  /**@brief The flag of pool auto expand. */
  bool is_fixed_size_;
  /**@brief The list for free node. */
  std::list<BaseMapNode*> free_list_;
  /**@brief The set for used node. */
  std::set<BaseMapNode*> busy_nodes_;
  /**@brief The size of memory pool. */
  unsigned int pool_size_;
  /**@brief The thread pool for release node. */
  ThreadPool node_reset_workers_;
  /**@brief The mutex for release thread.*/
  boost::mutex mutex_;
  /**@brief The mutex for release thread.*/
  const BaseMapConfig* map_config_;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_MSF_LOCAL_MAP_BASE_MAP_BASE_MAP_POOL_H
