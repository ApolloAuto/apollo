/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "cyber/task/task.h"
#include "modules/localization/msf/common/util/base_map_cache.h"
#include "modules/localization/msf/local_pyramid_map/base_map/base_map_config.h"
#include "modules/localization/msf/local_pyramid_map/base_map/base_map_fwd.h"
#include "modules/localization/msf/local_pyramid_map/base_map/base_map_node.h"
#include "modules/localization/msf/local_pyramid_map/base_map/base_map_node_index.h"
#include "modules/localization/msf/local_pyramid_map/base_map/base_map_pool.h"

namespace apollo {
namespace localization {
namespace msf {
namespace pyramid_map {

/**@brief The data structure of the base map. */
class BaseMap {
 public:
  /**@brief The constructor. */
  explicit BaseMap(BaseMapConfig* config);
  /**@brief The destructor. */
  virtual ~BaseMap();

  // @brief Init level 1 and level 2 map node caches. */
  virtual void InitMapNodeCaches(int cacheL1_size, int cahceL2_size);

  /**@brief Attach map node pointer. */
  void AttachMapNodePool(BaseMapNodePool* p_map_node_pool);

  /**@brief Return the map node, if it's not in the cache, return false. */
  BaseMapNode* GetMapNode(const MapNodeIndex& index);

  /**@brief Return the map node, if it's not in the cache, safely load it. */
  BaseMapNode* GetMapNodeSafe(const MapNodeIndex& index);

  /**@brief Check if the map node in the cache. */
  bool IsMapNodeExist(const MapNodeIndex& index);

  /**@brief Set the directory of the map. */
  bool SetMapFolderPath(const std::string folder_path);

  /**@brief Add a dataset path to the map config. */
  void AddDataset(const std::string dataset_path);

  /**@brief Release resources. */
  void Release();

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
   * finish,
   * in order to the nodes which the following calculate needed are all in the
   * memory, eigen version. */
  virtual bool LoadMapArea(const Eigen::Vector3d& seed_pt3d,
                           unsigned int resolution_id, unsigned int zone_id,
                           int filter_size_x, int filter_size_y);

  /**@brief Compute md5 for all map node file in map. */
  void ComputeMd5ForAllMapNodes();

  /**@brief Check if map is normal. */
  bool CheckMap();
  /**@brief Check if map is normal(with map node checking). */
  bool CheckMapStrictly();

  /**@brief Get the map config. */
  inline const BaseMapConfig& GetMapConfig() const { return *map_config_; }
  /**@brief Get the map config. */
  inline BaseMapConfig& GetMapConfig() { return *map_config_; }
  /**@brief Get all map node paths. */
  inline const std::vector<std::string>& GetAllMapNodePaths() const {
    return all_map_node_paths_;
  }
  /**@brief Get all map node md5s. */
  inline const std::vector<std::string>& GetAllMapNodeMd5s() const {
    return all_map_node_md5s_;
  }

 protected:
  void GetAllMapIndexAndPath();
  MapNodeIndex GetMapIndexFromMapPath(const std::string& map_path);

 protected:
  /**@brief Load map node by index.*/
  void LoadMapNodes(std::set<MapNodeIndex>* map_ids);
  /**@brief Load map node by index.*/
  void PreloadMapNodes(std::set<MapNodeIndex>* map_ids);
  /**@brief Load map node by index, thread_safety. */
  void LoadMapNodeThreadSafety(const MapNodeIndex& index,
                               bool is_reserved = false);
  /**@brief Check map node in L2 Cache.*/
  void CheckAndUpdateCache(std::set<MapNodeIndex>* map_ids);

  /**@brief The map settings. */
  BaseMapConfig* map_config_ = nullptr;

  MapNodeCache<MapNodeIndex, BaseMapNode>::DestroyFunc destroy_func_lvl1_;
  MapNodeCache<MapNodeIndex, BaseMapNode>::DestroyFunc destroy_func_lvl2_;
  /**@brief The cache for map node preload. */
  std::unique_ptr<MapNodeCache<MapNodeIndex, BaseMapNode>>
      map_node_cache_lvl1_ = nullptr;
  /**brief The dynamic map node preloading thread pool pointer. */
  std::unique_ptr<MapNodeCache<MapNodeIndex, BaseMapNode>>
      map_node_cache_lvl2_ = nullptr;
  /**@brief The map node memory pool pointer. */
  BaseMapNodePool* map_node_pool_ = nullptr;
  /**@bried Keep the index of preloading nodes. */
  std::set<MapNodeIndex> map_preloading_task_index_;
  /**@brief The mutex for preload map node. **/
  boost::recursive_mutex map_load_mutex_;

  /**@brief All the map nodes in the Map (in the disk). */
  std::vector<MapNodeIndex> all_map_node_indices_;
  std::vector<std::string> all_map_node_paths_;

  /**@brief All the map nodes' md5 in the Map (in the disk). */
  std::vector<std::string> all_map_node_md5s_;
};

}  // namespace pyramid_map
}  // namespace msf
}  // namespace localization
}  // namespace apollo
