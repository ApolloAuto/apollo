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

#include <boost/filesystem.hpp>

#include "gtest/gtest.h"

#include "modules/localization/msf/local_pyramid_map/pyramid_map/pyramid_map.h"
#include "modules/localization/msf/local_pyramid_map/pyramid_map/pyramid_map_config.h"
#include "modules/localization/msf/local_pyramid_map/pyramid_map/pyramid_map_pool.h"

#include "modules/localization/msf/local_pyramid_map/base_map/base_map.h"
#include "modules/localization/msf/local_pyramid_map/base_map/base_map_config.h"
#include "modules/localization/msf/local_pyramid_map/base_map/base_map_pool.h"

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

namespace apollo {
namespace localization {
namespace msf {
namespace pyramid_map {

class PyramidMapPoolTestSuite : public ::testing::Test {
 protected:
  PyramidMapPoolTestSuite() {}
  virtual ~PyramidMapPoolTestSuite() {}
  virtual void SetUp() {}
  virtual void TearDown() {}
};

TEST_F(PyramidMapPoolTestSuite, pyramid_map_pool_fixed) {
  // init config
  std::unique_ptr<PyramidMapConfig> config(
      new PyramidMapConfig("lossy_full_alt"));
  config->SetMapNodeSize(2, 2);
  config->resolution_num_ = 1;
  config->map_folder_path_ = "test_map_pool";

  std::unique_ptr<PyramidMapNodePool> pool(new PyramidMapNodePool(2, 3));
  pool->Initial(config.get());
  EXPECT_EQ(pool->GetPoolSize(), 2);

  // alloc map node
  BaseMapNode* node1 = pool->AllocMapNode();
  BaseMapNode* node2 = pool->AllocMapNode();
  BaseMapNode* node3 = pool->AllocMapNode();
  EXPECT_EQ(node3, nullptr);

  // free node
  pool->FreeMapNode(node2);
  pool->FreeMapNode(node1);

  // Release
  pool->Release();
  EXPECT_EQ(pool->GetPoolSize(), 0);
  pool.reset();
}

TEST_F(PyramidMapPoolTestSuite, pyramid_map_pool_not_fixed) {
  // init config
  std::unique_ptr<PyramidMapConfig> config(
      new PyramidMapConfig("lossy_full_alt"));
  config->SetMapNodeSize(2, 2);
  config->resolution_num_ = 1;
  config->map_folder_path_ = "test_map_pool";

  std::unique_ptr<PyramidMapNodePool> pool(new PyramidMapNodePool(2, 3));
  pool->Initial(config.get(), false);
  EXPECT_EQ(pool->GetPoolSize(), 2);

  // alloc map node
  BaseMapNode* node1 = pool->AllocMapNode();
  BaseMapNode* node2 = pool->AllocMapNode();
  BaseMapNode* node3 = pool->AllocMapNode();
  EXPECT_NE(node3, nullptr);
  EXPECT_EQ(pool->GetPoolSize(), 3);

  pool->FreeMapNode(node2);
  pool->FreeMapNode(node1);

  // Release
  pool->Release();
  EXPECT_EQ(pool->GetPoolSize(), 0);
  pool.reset();
}

}  // namespace pyramid_map
}  // namespace msf
}  // namespace localization
}  // namespace apollo
