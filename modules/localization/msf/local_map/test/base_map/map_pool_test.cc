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

#include <gtest/gtest.h>
#include "modules/localization/msf/local_map/lossless_map/lossless_map_config.h"
#include "modules/localization/msf/local_map/lossless_map/lossless_map_pool.h"

namespace apollo {
namespace localization {
namespace msf {

class BaseMapPoolTestSuite : public ::testing::Test {
 protected:
  BaseMapPoolTestSuite() {}
  virtual ~BaseMapPoolTestSuite() {}
  virtual void SetUp() {}
  virtual void TearDown() {}
};

/**@brief Test all public methods in BaseMapNodePool.*/
TEST_F(BaseMapPoolTestSuite, MapNodePoolTest) {
  LosslessMapConfig option;
  bool is_fixed_size = false;

  LosslessMapNodePool pool(3, 3);
  pool.Initial(&option, is_fixed_size);

  BaseMapNode* node1 = pool.AllocMapNode();
  BaseMapNode* node2 = pool.AllocMapNode();
  BaseMapNode* node3 = pool.AllocMapNode();
  BaseMapNode* node4 = pool.AllocMapNode();
  pool.FreeMapNode(node1);
  pool.FreeMapNode(node2);
  pool.FreeMapNode(node3);
  pool.FreeMapNode(node4);

  unsigned int pool_size = pool.GetPoolSize();
  ASSERT_EQ(pool_size, 4);

  pool.Release();
  pool_size = pool.GetPoolSize();
  ASSERT_EQ(pool_size, 0);
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
