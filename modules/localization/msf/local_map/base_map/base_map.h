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

#include <list>
#include <set>
#include <string>

#include "modules/localization/msf/common/util/base_map_cache.h"
#include "modules/localization/msf/local_map/base_map/base_map_config.h"
#include "modules/localization/msf/local_map/base_map/base_map_fwd.h"
#include "modules/localization/msf/local_map/base_map/base_map_node.h"
#include "modules/localization/msf/local_map/base_map/base_map_node_index.h"
#include "modules/localization/msf/local_map/base_map/base_map_pool.h"

namespace apollo {
namespace localization {
namespace msf {

/**@brief The data structure of the base map. */
class BaseMap {
 public:
  /**@brief The constructor. */
  explicit BaseMap(BaseMapConfig* map_config);
  /**@brief The destructor. */
  virtual ~BaseMap();

  /**@brief Init load threadpool and preload threadpool. */
  virtual void InitMapNodeCaches(int cacheL1_size, int cahceL2_size);

  /**@brief Get the map node, if it's not in the cache, return false. */
  BaseMapNode* GetMapNode(const MapNodeIndex& index);
  /**@brief Return the map node, if it's not in the cache, safely load it. */
  BaseMapNode* GetMapNodeSafe(const MapNodeIndex& index);
  /**@brief Check if the map node in the cache. */
  bool IsMapNodeExist(const MapNodeIndex& index) const;

  /**@brief Set the directory of the map. */
  bool SetMapFolderPath(const std::string folder_path);
  /**@brief Add a dataset path to the map config. */
  void AddDataset(const std::string dataset_path);

  /**@brief Preload map nodes for the next frame location calculation.
   * It will forecasts the nodes by the direction of the car moving.
   * Because the progress of loading will cost a long time (over 100ms),
   * it must do this for a period of time in advance.
   * After the index of nodes calculate finished, it will create loading tasks,
   * but will not wait for the loading finished, eigen version. */
  virtual void PreloadMapArea(const Eigen::Vector3d& location,
                              const Eigen::Vector3d& trans_diff,
                              unsigned int resolution_id, unsigned int zone_id);
  /**@brief Load map nodes for the location calculate of this frame.
   * If the forecasts are correct in last frame, these nodes will be all in
   * cache, if not, then need to create loading tasks, and wait for the loading
   * finish, in order to the nodes which the following calculate needed are all
   * in the memory, eigen version. */
  virtual bool LoadMapArea(const Eigen::Vector3d& seed_pt3d,
                           unsigned int resolution_id, unsigned int zone_id,
                           int filter_size_x, int filter_size_y);

  /**@brief Attach map node pointer. */
  void AttachMapNodePool(BaseMapNodePool* p_map_node_pool);

  /**@brief Write all the map nodes to a single binary file stream. It's for
   * binary streaming or packing.
   * @param <buf, buf_size> The buffer and its size.
   * @param <return> The required or the used size of is returned.
   */
  void WriteBinary(FILE* file);
  /**@brief Load all the map nodes from a single binary file stream. It's for
   * binary streaming or packing.
   * @param <map_folder_path> The new map folder path to save the map.
   * @param <return> The size read (the real size of body).
   */
  void LoadBinary(FILE* file, std::string map_folder_path = "");

  /**@brief Get the map config. */
  inline const BaseMapConfig& GetConfig() const { return *map_config_; }
  /**@brief Get the map config. */
  inline BaseMapConfig& GetConfig() { return *map_config_; }

 protected:
  /**@brief Load map node by index.*/
  void LoadMapNodes(std::set<MapNodeIndex>* map_ids);
  /**@brief Load map node by index.*/
  void PreloadMapNodes(std::set<MapNodeIndex>* map_ids);
  /**@brief Load map node by index, thread_safety. */
  void LoadMapNodeThreadSafety(MapNodeIndex index, bool is_reserved = false);

  /**@brief The map settings. */
  BaseMapConfig* map_config_;
  /**@brief All the map nodes in the Map (in the disk). */
  std::list<MapNodeIndex> map_nodes_disk_;

  MapNodeCache<MapNodeIndex, BaseMapNode>::DestroyFunc destroy_func_lvl1_;
  MapNodeCache<MapNodeIndex, BaseMapNode>::DestroyFunc destroy_func_lvl2_;
  /**@brief The cache for map node preload. */
  MapNodeCache<MapNodeIndex, BaseMapNode>* map_node_cache_lvl1_;
  /**brief The dynamic map node preloading thread pool pointer. */
  MapNodeCache<MapNodeIndex, BaseMapNode>* map_node_cache_lvl2_;
  /**@brief The map node memory pool pointer. */
  BaseMapNodePool* map_node_pool_;
  /**@bried Keep the index of preloading nodes. */
  std::set<MapNodeIndex> map_preloading_task_index_;
  /**@brief The mutex for preload map node. **/
  boost::recursive_mutex map_load_mutex_;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
