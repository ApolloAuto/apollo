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

#include "modules/localization/msf/local_map/base_map/base_map.h"

#include <vector>

#include "cyber/common/log.h"
#include "cyber/task/task.h"
#include "modules/localization/msf/common/util/system_utility.h"

namespace apollo {
namespace localization {
namespace msf {

BaseMap::BaseMap(BaseMapConfig* map_config)
    : map_config_(map_config),
      map_node_cache_lvl1_(nullptr),
      map_node_cache_lvl2_(nullptr),
      map_node_pool_(nullptr) {}

BaseMap::~BaseMap() {
  if (map_node_cache_lvl1_) {
    delete map_node_cache_lvl1_;
    map_node_cache_lvl1_ = nullptr;
  }
  if (map_node_cache_lvl2_) {
    delete map_node_cache_lvl2_;
    map_node_cache_lvl2_ = nullptr;
  }
}

void BaseMap::InitMapNodeCaches(int cacheL1_size, int cahceL2_size) {
  CHECK(map_node_cache_lvl1_ == nullptr);
  CHECK(map_node_cache_lvl2_ == nullptr);
  map_node_cache_lvl1_ =
      new MapNodeCacheL1<MapNodeIndex, BaseMapNode>(cacheL1_size);
  map_node_cache_lvl2_ =
      new MapNodeCacheL2<MapNodeIndex, BaseMapNode>(cahceL2_size);
}

BaseMapNode* BaseMap::GetMapNode(const MapNodeIndex& index) {
  BaseMapNode* node = nullptr;
  map_node_cache_lvl1_->Get(index, &node);
  return node;
}

BaseMapNode* BaseMap::GetMapNodeSafe(const MapNodeIndex& index) {
  BaseMapNode* node = nullptr;
  // try get from cacheL1
  if (map_node_cache_lvl1_->Get(index, &node)) {
    return node;
  }

  // try get from cacheL2
  boost::unique_lock<boost::recursive_mutex> lock(map_load_mutex_);
  if (map_node_cache_lvl2_->Get(index, &node)) {
    node->SetIsReserved(true);

    map_node_cache_lvl1_->Put(index, node);
    return node;
  }
  lock.unlock();

  // load from disk
  AERROR << "GetMapNodeSafe: This node don't exist in cache! ";
  AERROR << "load this node from disk now! index = " << index;
  LoadMapNodeThreadSafety(index, true);
  boost::unique_lock<boost::recursive_mutex> lock2(map_load_mutex_);
  map_node_cache_lvl2_->Get(index, &node);
  lock2.unlock();

  map_node_cache_lvl1_->Put(index, node);
  return node;
}

/**@brief Check if the map node in the cache. */
bool BaseMap::IsMapNodeExist(const MapNodeIndex& index) const {
  return map_node_cache_lvl1_->IsExist(index);
}

bool BaseMap::SetMapFolderPath(const std::string folder_path) {
  map_config_->map_folder_path_ = folder_path;

  // Try to load the config
  std::string config_path = map_config_->map_folder_path_ + "/config.xml";
  if (system::IsExists(config_path)) {
    map_config_->Load(config_path);
    return true;
  } else {
    return false;
  }
}

void BaseMap::AddDataset(const std::string dataset_path) {
  map_config_->map_datasets_.push_back(dataset_path);
  std::string config_path = map_config_->map_folder_path_ + "/config.xml";
  map_config_->Save(config_path);
}

void BaseMap::LoadMapNodes(std::set<MapNodeIndex>* map_ids) {
  CHECK_LE(static_cast<int>(map_ids->size()), map_node_cache_lvl1_->Capacity());
  // check in cacheL1
  typename std::set<MapNodeIndex>::iterator itr = map_ids->begin();
  while (itr != map_ids->end()) {
    if (map_node_cache_lvl1_->IsExist(*itr)) {
      // std::cout << "LoadMapNodes find in L1 cache" << std::endl;
      boost::unique_lock<boost::recursive_mutex> lock(map_load_mutex_);
      map_node_cache_lvl2_->IsExist(*itr);  // fresh lru list
      lock.unlock();
      itr = map_ids->erase(itr);
    } else {
      ++itr;
    }
  }

  // check in cacheL2
  itr = map_ids->begin();
  BaseMapNode* node = nullptr;
  boost::unique_lock<boost::recursive_mutex> lock(map_load_mutex_);
  while (itr != map_ids->end()) {
    if (map_node_cache_lvl2_->Get(*itr, &node)) {
      // std::cout << "LoadMapNodes find in L2 cache" << std::endl;
      node->SetIsReserved(true);
      map_node_cache_lvl1_->Put(*itr, node);
      itr = map_ids->erase(itr);
    } else {
      ++itr;
    }
  }
  lock.unlock();

  // load from disk sync
  std::vector<std::future<void>> load_futures;
  itr = map_ids->begin();
  while (itr != map_ids->end()) {
    AERROR << "Preload map node failed!";
    load_futures.emplace_back(
        cyber::Async(&BaseMap::LoadMapNodeThreadSafety, this, *itr, true));
    ++itr;
  }

  for (auto& future : load_futures) {
    if (future.valid()) {
      future.get();
    }
  }
  // check in cacheL2 again
  itr = map_ids->begin();
  node = nullptr;
  boost::unique_lock<boost::recursive_mutex> lock2(map_load_mutex_);
  while (itr != map_ids->end()) {
    if (map_node_cache_lvl2_->Get(*itr, &node)) {
      AINFO << "LoadMapNodes: preload missed, load this node in main thread.\n"
            << *itr;
      node->SetIsReserved(true);
      map_node_cache_lvl1_->Put(*itr, node);
      itr = map_ids->erase(itr);
    } else {
      ++itr;
    }
  }
  lock2.unlock();

  CHECK(map_ids->empty());
}

void BaseMap::PreloadMapNodes(std::set<MapNodeIndex>* map_ids) {
  DCHECK_LE(static_cast<int>(map_ids->size()),
            map_node_cache_lvl2_->Capacity());
  // check in cacheL2
  typename std::set<MapNodeIndex>::iterator itr = map_ids->begin();
  while (itr != map_ids->end()) {
    boost::unique_lock<boost::recursive_mutex> lock(map_load_mutex_);
    bool is_exist = map_node_cache_lvl2_->IsExist(*itr);
    lock.unlock();
    if (is_exist) {
      itr = map_ids->erase(itr);
    } else {
      ++itr;
    }
  }

  // check whether in already preloading index set
  itr = map_ids->begin();
  auto preloading_itr = map_preloading_task_index_.end();
  while (itr != map_ids->end()) {
    boost::unique_lock<boost::recursive_mutex> lock(map_load_mutex_);
    preloading_itr = map_preloading_task_index_.find(*itr);
    lock.unlock();
    if (preloading_itr !=
        map_preloading_task_index_.end()) {  // already preloading
      itr = map_ids->erase(itr);
    } else {
      ++itr;
    }
  }

  // load form disk sync
  std::vector<std::future<void>> preload_futures;
  itr = map_ids->begin();
  AINFO << "Preload map node size: " << map_ids->size();
  while (itr != map_ids->end()) {
    AINFO << "Preload map node: " << *itr << std::endl;
    boost::unique_lock<boost::recursive_mutex> lock3(map_load_mutex_);
    map_preloading_task_index_.insert(*itr);
    lock3.unlock();
    preload_futures.emplace_back(
        cyber::Async(&BaseMap::LoadMapNodeThreadSafety, this, *itr, false));
    ++itr;
  }
}

void BaseMap::AttachMapNodePool(BaseMapNodePool* map_node_pool) {
  map_node_pool_ = map_node_pool;
}

void BaseMap::LoadMapNodeThreadSafety(MapNodeIndex index, bool is_reserved) {
  BaseMapNode* map_node = nullptr;
  while (map_node == nullptr) {
    map_node = map_node_pool_->AllocMapNode();
    if (map_node == nullptr) {
      boost::unique_lock<boost::recursive_mutex> lock(map_load_mutex_);
      BaseMapNode* node_remove = map_node_cache_lvl2_->ClearOne();
      if (node_remove) {
        map_node_pool_->FreeMapNode(node_remove);
      }
    }
  }
  map_node->Init(map_config_, index, false);
  if (!map_node->Load()) {
    AERROR << "Created map node: " << index;
  } else {
    AERROR << " Loaded map node: " << index;
  }
  map_node->SetIsReserved(is_reserved);

  boost::unique_lock<boost::recursive_mutex> lock(map_load_mutex_);
  BaseMapNode* node_remove = map_node_cache_lvl2_->Put(index, map_node);
  // if the node already added into cacheL2, erase it from preloading set
  auto itr = map_preloading_task_index_.find(index);
  if (itr != map_preloading_task_index_.end()) {
    map_preloading_task_index_.erase(itr);
  }
  if (node_remove) {
    map_node_pool_->FreeMapNode(node_remove);
  }
}

void BaseMap::PreloadMapArea(const Eigen::Vector3d& location,
                             const Eigen::Vector3d& trans_diff,
                             unsigned int resolution_id, unsigned int zone_id) {
  CHECK_NOTNULL(map_node_pool_);

  int x_direction = trans_diff[0] > 0 ? 1 : -1;
  int y_direction = trans_diff[1] > 0 ? 1 : -1;

  std::set<MapNodeIndex> map_ids;
  float map_pixel_resolution =
      this->map_config_->map_resolutions_[resolution_id];
  /// top left
  Eigen::Vector3d pt_top_left;
  pt_top_left[0] =
      location[0] - (static_cast<float>(this->map_config_->map_node_size_x_) *
                     map_pixel_resolution / 2.0f);
  pt_top_left[1] =
      location[1] - (static_cast<float>(this->map_config_->map_node_size_y_) *
                     map_pixel_resolution / 2.0f);
  pt_top_left[2] = 0;
  MapNodeIndex map_id = MapNodeIndex::GetMapNodeIndex(
      *(this->map_config_), pt_top_left, resolution_id, zone_id);
  map_ids.insert(map_id);

  /// top center
  Eigen::Vector3d pt_top_center;
  pt_top_center[0] = location[0];
  pt_top_center[1] = pt_top_left[1];
  pt_top_center[2] = 0;
  map_id = MapNodeIndex::GetMapNodeIndex(*(this->map_config_), pt_top_center,
                                         resolution_id, zone_id);
  map_ids.insert(map_id);

  /// top right
  Eigen::Vector3d pt_top_right;
  pt_top_right[0] =
      location[0] + (static_cast<float>(this->map_config_->map_node_size_x_) *
                     map_pixel_resolution / 2.0);
  pt_top_right[1] = pt_top_left[1];
  pt_top_right[2] = 0;
  map_id = MapNodeIndex::GetMapNodeIndex(*(this->map_config_), pt_top_right,
                                         resolution_id, zone_id);
  map_ids.insert(map_id);

  /// middle left
  Eigen::Vector3d pt_middle_left;
  pt_middle_left[0] = pt_top_left[0];
  pt_middle_left[1] = location[1];
  pt_middle_left[2] = 0;
  map_id = MapNodeIndex::GetMapNodeIndex(*(this->map_config_), pt_middle_left,
                                         resolution_id, zone_id);
  map_ids.insert(map_id);

  /// middle center
  map_id = MapNodeIndex::GetMapNodeIndex(*(this->map_config_), location,
                                         resolution_id, zone_id);
  map_ids.insert(map_id);

  /// middle right
  Eigen::Vector3d pt_middle_right;
  pt_middle_right[0] = pt_top_right[0];
  pt_middle_right[1] = pt_middle_left[1];
  pt_middle_right[2] = 0;
  map_id = MapNodeIndex::GetMapNodeIndex(*(this->map_config_), pt_middle_right,
                                         resolution_id, zone_id);
  map_ids.insert(map_id);

  /// bottom left
  Eigen::Vector3d pt_bottom_left;
  pt_bottom_left[0] = pt_top_left[0];
  pt_bottom_left[1] =
      location[1] + (static_cast<float>(this->map_config_->map_node_size_y_) *
                     map_pixel_resolution / 2.0);
  pt_bottom_left[2] = 0;
  map_id = MapNodeIndex::GetMapNodeIndex(*(this->map_config_), pt_bottom_left,
                                         resolution_id, zone_id);
  map_ids.insert(map_id);

  /// bottom center
  Eigen::Vector3d pt_bottom_center;
  pt_bottom_center[0] = pt_top_center[0];
  pt_bottom_center[1] = pt_bottom_left[1];
  pt_bottom_center[2] = 0;
  map_id = MapNodeIndex::GetMapNodeIndex(*(this->map_config_), pt_bottom_center,
                                         resolution_id, zone_id);
  map_ids.insert(map_id);

  /// bottom right
  Eigen::Vector3d pt_bottom_right;
  pt_bottom_right[0] = pt_top_right[0];
  pt_bottom_right[1] = pt_bottom_left[1];
  pt_bottom_right[2] = 0;
  map_id = MapNodeIndex::GetMapNodeIndex(*(this->map_config_), pt_bottom_right,
                                         resolution_id, zone_id);
  map_ids.insert(map_id);

  for (int i = -1; i < 2; ++i) {
    Eigen::Vector3d pt;
    pt[0] = location[0] + x_direction * 1.5 *
                              this->map_config_->map_node_size_x_ *
                              map_pixel_resolution;
    pt[1] = location[1] + static_cast<double>(i) *
                              this->map_config_->map_node_size_y_ *
                              map_pixel_resolution;
    pt[2] = 0;
    map_id = MapNodeIndex::GetMapNodeIndex(*(this->map_config_), pt,
                                           resolution_id, zone_id);
    map_ids.insert(map_id);
  }
  for (int i = -1; i < 2; ++i) {
    Eigen::Vector3d pt;
    pt[0] = location[0] + static_cast<double>(i) *
                              this->map_config_->map_node_size_x_ *
                              map_pixel_resolution;
    pt[1] = location[1] + y_direction * 1.5 *
                              this->map_config_->map_node_size_y_ *
                              map_pixel_resolution;
    pt[2] = 0;
    map_id = MapNodeIndex::GetMapNodeIndex(*(this->map_config_), pt,
                                           resolution_id, zone_id);
    map_ids.insert(map_id);
  }
  {
    Eigen::Vector3d pt;
    pt[0] = location[0] + x_direction * 1.5 *
                              this->map_config_->map_node_size_x_ *
                              map_pixel_resolution;
    pt[1] = location[1] + y_direction * 1.5 *
                              this->map_config_->map_node_size_y_ *
                              map_pixel_resolution;
    pt[2] = 0;
    map_id = MapNodeIndex::GetMapNodeIndex(*(this->map_config_), pt,
                                           resolution_id, zone_id);
    map_ids.insert(map_id);
  }

  this->PreloadMapNodes(&map_ids);
}

bool BaseMap::LoadMapArea(const Eigen::Vector3d& seed_pt3d,
                          unsigned int resolution_id, unsigned int zone_id,
                          int filter_size_x, int filter_size_y) {
  CHECK_NOTNULL(map_node_pool_);
  std::set<MapNodeIndex> map_ids;
  float map_pixel_resolution =
      this->map_config_->map_resolutions_[resolution_id];
  /// top left
  Eigen::Vector3d pt_top_left;
  pt_top_left[0] = seed_pt3d[0] -
                   (static_cast<float>(this->map_config_->map_node_size_x_) *
                    map_pixel_resolution / 2.0f) -
                   static_cast<float>(filter_size_x / 2) * map_pixel_resolution;
  pt_top_left[1] = seed_pt3d[1] -
                   (static_cast<float>(this->map_config_->map_node_size_y_) *
                    map_pixel_resolution / 2.0f) -
                   static_cast<float>(filter_size_y / 2) * map_pixel_resolution;
  pt_top_left[2] = 0;
  MapNodeIndex map_id = MapNodeIndex::GetMapNodeIndex(
      *(this->map_config_), pt_top_left, resolution_id, zone_id);
  map_ids.insert(map_id);

  /// top center
  Eigen::Vector3d pt_top_center;
  pt_top_center[0] = seed_pt3d[0];
  pt_top_center[1] = pt_top_left[1];
  pt_top_center[2] = 0;
  map_id = MapNodeIndex::GetMapNodeIndex(*(this->map_config_), pt_top_center,
                                         resolution_id, zone_id);
  map_ids.insert(map_id);

  /// top right
  Eigen::Vector3d pt_top_right;
  pt_top_right[0] =
      seed_pt3d[0] +
      (static_cast<float>(this->map_config_->map_node_size_x_) *
       map_pixel_resolution / 2.0f) +
      static_cast<float>(filter_size_x / 2) * map_pixel_resolution;
  pt_top_right[1] = pt_top_left[1];
  pt_top_left[2] = 0;
  map_id = MapNodeIndex::GetMapNodeIndex(*(this->map_config_), pt_top_right,
                                         resolution_id, zone_id);
  map_ids.insert(map_id);

  /// middle left
  Eigen::Vector3d pt_middle_left;
  pt_middle_left[0] = pt_top_left[0];
  pt_middle_left[1] = seed_pt3d[1];
  pt_middle_left[2] = 0;
  map_id = MapNodeIndex::GetMapNodeIndex(*(this->map_config_), pt_middle_left,
                                         resolution_id, zone_id);
  map_ids.insert(map_id);

  /// middle center
  map_id = MapNodeIndex::GetMapNodeIndex(*(this->map_config_), seed_pt3d,
                                         resolution_id, zone_id);
  map_ids.insert(map_id);

  /// middle right
  Eigen::Vector3d pt_middle_right;
  pt_middle_right[0] = pt_top_right[0];
  pt_middle_right[1] = seed_pt3d[1];
  pt_middle_right[2] = 0;
  map_id = MapNodeIndex::GetMapNodeIndex(*(this->map_config_), pt_middle_right,
                                         resolution_id, zone_id);
  map_ids.insert(map_id);

  /// bottom left
  Eigen::Vector3d pt_bottom_left;
  pt_bottom_left[0] = pt_top_left[0];
  pt_bottom_left[1] =
      seed_pt3d[1] +
      (static_cast<float>(this->map_config_->map_node_size_y_) *
       map_pixel_resolution / 2.0f) +
      static_cast<float>(filter_size_y / 2) * map_pixel_resolution;
  pt_bottom_left[2] = 0;
  map_id = MapNodeIndex::GetMapNodeIndex(*(this->map_config_), pt_bottom_left,
                                         resolution_id, zone_id);
  map_ids.insert(map_id);

  /// bottom center
  Eigen::Vector3d pt_bottom_center;
  pt_bottom_center[0] = seed_pt3d[0];
  pt_bottom_center[1] = pt_bottom_left[1];
  pt_bottom_center[2] = 0;
  map_id = MapNodeIndex::GetMapNodeIndex(*(this->map_config_), pt_bottom_center,
                                         resolution_id, zone_id);
  map_ids.insert(map_id);

  /// bottom right
  Eigen::Vector3d pt_bottom_right;
  pt_bottom_right[0] = pt_top_right[0];
  pt_bottom_right[1] = pt_bottom_left[1];
  pt_bottom_right[2] = 0;
  map_id = MapNodeIndex::GetMapNodeIndex(*(this->map_config_), pt_bottom_right,
                                         resolution_id, zone_id);
  map_ids.insert(map_id);

  this->LoadMapNodes(&map_ids);
  return true;
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
