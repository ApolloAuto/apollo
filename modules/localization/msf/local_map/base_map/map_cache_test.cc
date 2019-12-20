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

#include "gtest/gtest.h"

#include "modules/localization/msf/local_map/base_map/base_map_cache.h"
#include "modules/localization/msf/local_map/base_map/base_map_node_index.h"

namespace apollo {
namespace localization {
namespace msf {

class MapCacheTestSuite : public ::testing::Test {
 protected:
  MapCacheTestSuite() {}
  virtual ~MapCacheTestSuite() {}
  virtual void SetUp() {}
  virtual void TearDown() {}
};

/**@brief Test LRUCache. */
TEST_F(MapCacheTestSuite, LRUCacheTest) {
  LRUCache<unsigned int, std::string> lru_cache(3);

  unsigned int id0 = 0;
  unsigned int id1 = 1;
  unsigned int id2 = 2;
  unsigned int id3 = 3;
  unsigned int id4 = 4;

  std::string *ch0 = new std::string("aaa");
  std::string *ch1 = new std::string("bbb");
  std::string *ch2 = new std::string("ccc");
  std::string *ch3 = new std::string("ddd");
  std::string *ch4 = new std::string("eee");

  lru_cache.Put(id0, ch0);
  lru_cache.Put(id1, ch1);
  lru_cache.Put(id2, ch2);

  std::string *str = lru_cache.Put(id3, ch3);
  ASSERT_EQ(strcmp(str->c_str(), "aaa"), 0);

  bool flag = lru_cache.Get(id3, &str);
  ASSERT_EQ(strcmp(str->c_str(), "ddd"), 0);
  ASSERT_TRUE(flag);

  str = lru_cache.Remove(id3);
  ASSERT_EQ(strcmp(str->c_str(), "ddd"), 0);

  lru_cache.Put(id4, ch4);
  flag = lru_cache.IsExist(id4);
  ASSERT_TRUE(flag);

  flag = lru_cache.ChangeCapacity(2);
  ASSERT_FALSE(flag);

  flag = lru_cache.ChangeCapacity(3);
  ASSERT_TRUE(flag);

  int capacity = lru_cache.Capacity();
  ASSERT_EQ(capacity, 3);

  lru_cache.Put(id0, ch0);
  int size = lru_cache.Size();
  ASSERT_EQ(size, 3);

  str = lru_cache.ClearOne();
  ASSERT_TRUE(strcmp(str->c_str(), "bbb"));
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
