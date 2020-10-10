/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
#include "modules/localization/msf/common/util/base_map_cache.h"
#include <string>
#include <vector>
#include "gtest/gtest.h"

namespace apollo {
namespace localization {
namespace msf {

class NodeIndex {
 public:
  NodeIndex() : m_(0), n_(0) {}
  NodeIndex(int m, int n) : m_(m), n_(n) {}

 public:
  bool operator<(const NodeIndex& index) const {
    return std::forward_as_tuple(m_, n_) <
           std::forward_as_tuple(index.m_, index.n_);
  }
  bool operator==(const NodeIndex& index) const {
    return m_ == index.m_ && n_ == index.n_;
  }
  bool operator!=(const NodeIndex& index) const {
    return m_ != index.m_ || n_ != index.n_;
  }

 private:
  int m_;
  int n_;
};

class NodeData {
 public:
  NodeData() : is_reserved_(false), name_("") {}
  explicit NodeData(const std::string& name)
      : is_reserved_(false), name_(name) {}

 public:
  void SetIsReserved(bool b) { is_reserved_ = b; }
  bool GetIsReserved() { return is_reserved_; }

  void SetName(const std::string& name) { name_ = name; }
  std::string GetName() { return name_; }

 private:
  bool is_reserved_;
  std::string name_;
};

class MapNodeCacheTest : public ::testing::Test {
 public:
  virtual void SetUp() { Init(); }

 protected:
  void Init() {
    destroy_func_lvl1_ =
        std::bind(MapNodeCache<NodeIndex, NodeData>::CacheL1Destroy,
                  std::placeholders::_1);
    destroy_func_lvl2_ =
        std::bind(MapNodeCache<NodeIndex, NodeData>::CacheL2Destroy,
                  std::placeholders::_1);

    map_node_cache_lvl_.reset(
        new MapNodeCache<NodeIndex, NodeData>(3, destroy_func_lvl1_));

    std::string name_a("aaa");
    std::string name_b("bbb");
    std::string name_c("ccc");
    std::string name_d("ddd");

    node_pool_.push_back(std::make_pair(NodeIndex(1, 2), NodeData(name_a)));
    node_pool_.push_back(std::make_pair(NodeIndex(2, 3), NodeData(name_b)));
    node_pool_.push_back(std::make_pair(NodeIndex(3, 4), NodeData(name_c)));
    node_pool_.push_back(std::make_pair(NodeIndex(4, 5), NodeData(name_d)));
  }

  MapNodeCache<NodeIndex, NodeData>::DestroyFunc destroy_func_lvl1_;
  MapNodeCache<NodeIndex, NodeData>::DestroyFunc destroy_func_lvl2_;
  std::unique_ptr<MapNodeCache<NodeIndex, NodeData>> map_node_cache_lvl_;

  std::vector<std::pair<NodeIndex, NodeData>> node_pool_;
};

TEST_F(MapNodeCacheTest, PutAndGet) {
  map_node_cache_lvl_->Put(node_pool_[0].first, &(node_pool_[0].second));
  map_node_cache_lvl_->Put(node_pool_[1].first, &(node_pool_[1].second));
  EXPECT_EQ(map_node_cache_lvl_->Size(), 2);
  NodeData* node_data = nullptr;
  map_node_cache_lvl_->ClearOne();
  EXPECT_EQ(map_node_cache_lvl_->Size(), 1);
  map_node_cache_lvl_->Get(std::move(NodeIndex(2, 3)), &node_data);
  EXPECT_EQ(node_data->GetName().compare("bbb"), 0);
}

TEST_F(MapNodeCacheTest, GetSilent) {
  map_node_cache_lvl_->Put(node_pool_[0].first, &(node_pool_[0].second));
  map_node_cache_lvl_->Put(node_pool_[1].first, &(node_pool_[1].second));
  EXPECT_EQ(map_node_cache_lvl_->Size(), 2);
  NodeData* node_data;
  map_node_cache_lvl_->GetSilent(std::move(NodeIndex(2, 3)), &node_data);
  map_node_cache_lvl_->ClearOne();
  EXPECT_EQ(map_node_cache_lvl_->Size(), 1);
  map_node_cache_lvl_->Get(std::move(NodeIndex(2, 3)), &node_data);
  EXPECT_EQ(node_data->GetName().compare("bbb"), 0);
}

TEST_F(MapNodeCacheTest, IsExist) {
  map_node_cache_lvl_->Put(node_pool_[0].first, &(node_pool_[0].second));
  map_node_cache_lvl_->Put(node_pool_[1].first, &(node_pool_[1].second));
  EXPECT_EQ(map_node_cache_lvl_->Size(), 2);
  NodeData* node_data;
  EXPECT_EQ(map_node_cache_lvl_->IsExist(std::move(NodeIndex(1, 2))), true);
  map_node_cache_lvl_->ClearOne();
  EXPECT_EQ(map_node_cache_lvl_->Size(), 1);
  map_node_cache_lvl_->Get(std::move(NodeIndex(1, 2)), &node_data);
  EXPECT_EQ(node_data->GetName().compare("aaa"), 0);
}

TEST_F(MapNodeCacheTest, DestroyFunc) {
  map_node_cache_lvl_->Put(node_pool_[0].first, &(node_pool_[0].second));
  map_node_cache_lvl_->Put(node_pool_[1].first, &(node_pool_[1].second));
  EXPECT_EQ(map_node_cache_lvl_->Size(), 2);
  NodeData* node_data;
  map_node_cache_lvl_->Get(std::move(NodeIndex(1, 2)), &node_data);
  EXPECT_EQ(node_data->GetName().compare("aaa"), 0);
  node_data->SetIsReserved(true);
  EXPECT_EQ(destroy_func_lvl2_(node_data), false);
  EXPECT_EQ(destroy_func_lvl1_(node_data), true);
  EXPECT_EQ(destroy_func_lvl2_(node_data), true);
}

TEST_F(MapNodeCacheTest, PutOverCapacity) {
  map_node_cache_lvl_->Put(node_pool_[0].first, &(node_pool_[0].second));
  map_node_cache_lvl_->Put(node_pool_[1].first, &(node_pool_[1].second));
  map_node_cache_lvl_->Put(node_pool_[2].first, &(node_pool_[2].second));
  map_node_cache_lvl_->Put(node_pool_[3].first, &(node_pool_[3].second));
  EXPECT_EQ(map_node_cache_lvl_->Size(), 3);
  EXPECT_EQ(map_node_cache_lvl_->IsExist(std::move(NodeIndex(1, 2))), false);
  EXPECT_EQ(map_node_cache_lvl_->IsExist(std::move(NodeIndex(2, 3))), true);
  EXPECT_EQ(map_node_cache_lvl_->IsExist(std::move(NodeIndex(3, 4))), true);
  EXPECT_EQ(map_node_cache_lvl_->IsExist(std::move(NodeIndex(4, 5))), true);
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
