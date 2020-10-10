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

#include "modules/common/util/lru_cache.h"

#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace common {
namespace util {

static const int TEST_NUM = 10;
static const int CAPACITY = 4;

TEST(LRUCache, General) {
  int ids[] = {0, 1, 2, 3, 2, 1, 4, 3, 5, 6};
  int timestamps[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

  std::vector<std::vector<int>> keys = {
      {0},          {1, 0},       {2, 1, 0},    {3, 2, 1, 0}, {2, 3, 1, 0},
      {1, 2, 3, 0}, {4, 1, 2, 3}, {3, 4, 1, 2}, {5, 3, 4, 1}, {6, 5, 3, 4}};
  std::vector<std::vector<int>> values = {
      {0},          {1, 0},       {2, 1, 0},    {3, 2, 1, 0}, {4, 3, 1, 0},
      {5, 4, 3, 0}, {6, 5, 4, 3}, {7, 6, 5, 4}, {8, 7, 6, 5}, {9, 8, 7, 6}};
  int obsoletes[TEST_NUM] = {-1, -1, -1, -1, -1, -1, 0, -1, 2, 1};
  LRUCache<int, int> lru(CAPACITY);
  for (int i = 0; i < TEST_NUM; ++i) {
    int obsolete = -1;
    lru.PutAndGetObsolete(ids[i], &timestamps[i], &obsolete);
    EXPECT_EQ(obsolete, obsoletes[i]);
    EXPECT_EQ(static_cast<int>(lru.size()), i < CAPACITY ? i + 1 : CAPACITY);

    Node<int, int>* cur = lru.First();
    for (int j = 0; j < static_cast<int>(lru.size()); ++j) {
      EXPECT_EQ(cur->key, keys[i][j]);
      EXPECT_EQ(cur->val, values[i][j]);
      cur = cur->next;
    }
  }
}

TEST(LRUCache, UAF) {
  LRUCache<int, int> cache;
  std::vector<int> keys = {1, 3, 5};
  std::vector<int> vals = {2, 4, 6};

  for (size_t i = 0; i < 2; ++i) {
    cache.Put(keys[i], vals[i]);
  }
  EXPECT_EQ(2, cache.size());
  Node<int, int>* curr = cache.First();
  EXPECT_NE(curr, nullptr);
  for (size_t i = 0; i < 2; ++i) {
    EXPECT_EQ(curr->key, keys[1 - i]);
    EXPECT_EQ(curr->val, vals[1 - i]);
    curr = curr->next;
  }
  cache.Clear();

  cache.Put(keys[2], vals[2]);
  EXPECT_EQ(1, cache.size());
  curr = cache.First();
  EXPECT_NE(curr, nullptr);
  for (int i = 2; i < 3; ++i) {
    EXPECT_EQ(curr->key, keys[i]);
    EXPECT_EQ(curr->val, vals[i]);
    curr = curr->next;
  }
  cache.Clear();
}

}  // namespace util
}  // namespace common
}  // namespace apollo
