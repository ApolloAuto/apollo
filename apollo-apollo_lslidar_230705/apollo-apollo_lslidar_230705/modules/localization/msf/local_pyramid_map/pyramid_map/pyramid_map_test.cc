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

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

namespace apollo {
namespace localization {
namespace msf {
namespace pyramid_map {

class PyramidMapTestSuite : public ::testing::Test {
 protected:
  PyramidMapTestSuite() {}
  virtual ~PyramidMapTestSuite() {}
  virtual void SetUp() {}
  virtual void TearDown() {}
};

void CreateTestMapNode(unsigned int m, unsigned int n,
                       const MapNodeIndex& index,
                       const PyramidMapConfig* config) {
  // create map node
  PyramidMapNode* node = new PyramidMapNode();
  node->Init(config);
  node->SetMapNodeIndex(index);

  // set map node data
  double data[] = {node->GetLeftTopCorner()[0] + 0.125,
                   node->GetLeftTopCorner()[1] + 0.125, 1.0};
  Eigen::Vector3d vec3d(data);
  EXPECT_TRUE(node->AddValueIfInBound(
      vec3d, static_cast<unsigned char>(m * config->map_node_size_x_ + n), 0));
  EXPECT_TRUE(node->Save());

  if (node != nullptr) {
    delete node;
    node = nullptr;
  }
}

TEST_F(PyramidMapTestSuite, pyramid_map_function) {
  // init config
  PyramidMapConfig* config = new PyramidMapConfig("lossy_full_alt");
  config->SetMapNodeSize(2, 2);
  config->resolution_num_ = 1;
  config->map_folder_path_ = "test_map";

  // create and save nodes
  unsigned int M = 4;
  unsigned int N = 4;
  std::vector<MapNodeIndex> indexes;
  for (unsigned int m = 0; m < M; ++m) {
    for (unsigned int n = 0; n < N; ++n) {
      // create map node index
      MapNodeIndex index;
      index.resolution_id_ = 0;
      index.zone_id_ = 50;
      index.m_ = m;
      index.n_ = n;
      CreateTestMapNode(m, n, index, config);
      indexes.push_back(index);
    }
  }
  // save config file
  config->Save("test_map/config.xml");

  // init pyramid map pool
  PyramidMapNodePool pm_node_pool(16, 4);
  pm_node_pool.Initial(config);

  // init pyramid map
  PyramidMap pyramid_map(config);
  pyramid_map.InitMapNodeCaches(4, 15);
  pyramid_map.AttachMapNodePool(&pm_node_pool);
  EXPECT_TRUE(pyramid_map.SetMapFolderPath(config->map_folder_path_));

  // load map node safe from disk
  PyramidMapNode* pm_node = dynamic_cast<PyramidMapNode*>(
      pyramid_map.GetMapNodeSafe(*indexes.begin()));
  double data[] = {0.125, 0.125, 1.0};
  Eigen::Vector3d loc(data);
  EXPECT_FLOAT_EQ(
      pm_node->GetIntensitySafe(loc),
      static_cast<float>(indexes.begin()->m_ * config->map_node_size_x_ +
                         indexes.begin()->n_));
  EXPECT_TRUE(pyramid_map.IsMapNodeExist(*indexes.begin()));
  EXPECT_FALSE(pyramid_map.IsMapNodeExist(*(indexes.begin() + 1)));

  // load map area
  loc[0] = 0.125 * (config->map_node_size_x_ + 1);
  loc[1] = 0.125 * (config->map_node_size_x_ + 1);
  loc[2] = 1.0;
  Vector3d loc_eigen;
  loc_eigen[0] = loc[0];
  loc_eigen[1] = loc[1];
  EXPECT_TRUE(pyramid_map.LoadMapArea(loc, 0, 50, 0, 0));
  EXPECT_TRUE(pyramid_map.LoadMapArea(loc_eigen, 0, 50, 0, 0));
  EXPECT_TRUE(pyramid_map.IsMapNodeExist(*(indexes.begin() + 5)));
  EXPECT_TRUE(pyramid_map.IsMapNodeExist(*(indexes.begin() + 6)));
  EXPECT_TRUE(pyramid_map.IsMapNodeExist(*(indexes.begin() + 9)));
  EXPECT_TRUE(pyramid_map.IsMapNodeExist(*(indexes.begin() + 10)));
  EXPECT_FALSE(pyramid_map.IsMapNodeExist(*(indexes.begin() + 0)));

  // preload map area
  Eigen::Vector3d trans_diff;
  trans_diff[0] = 1.0;
  trans_diff[1] = 1.0;
  trans_diff[2] = 1.0;
  pyramid_map.PreloadMapArea(loc, trans_diff, 0, 50);

  loc_eigen[0] = 0.125;
  loc_eigen[1] = 0.125;
  Vector3d trans_diff_eigen;
  trans_diff_eigen[0] = trans_diff[0];
  trans_diff_eigen[1] = trans_diff[1];
  trans_diff_eigen[2] = trans_diff[2];
  pyramid_map.PreloadMapArea(loc_eigen, trans_diff_eigen, 0, 50);

  // get path
  pyramid_map.ComputeMd5ForAllMapNodes();
  std::vector<std::string> paths = pyramid_map.GetAllMapNodePaths();
  EXPECT_EQ(paths.size(), M * N);
  std::vector<std::string> md5s = pyramid_map.GetAllMapNodeMd5s();
  EXPECT_EQ(md5s.size(), M * N);

  // set md5
  EXPECT_FALSE(pyramid_map.CheckMap());
  std::map<std::string, std::string> node_md5_map;
  for (unsigned int i = 0; i < paths.size(); ++i) {
    node_md5_map[paths[i]] = md5s[i];
  }
  config->SetNodeMd5Map(node_md5_map);
  EXPECT_TRUE(pyramid_map.CheckMap());

  // test PyramidMap function: access specific coordinate
  EXPECT_FLOAT_EQ(pyramid_map.GetIntensitySafe(loc, 50, 0),
                  1.f * static_cast<float>(config->map_node_size_x_) + 1.f);
  EXPECT_FLOAT_EQ(pyramid_map.GetIntensityVarSafe(loc, 50, 0), 0.f);
  EXPECT_FLOAT_EQ(pyramid_map.GetAltitudeSafe(loc, 50, 0), 1.f);
  EXPECT_FLOAT_EQ(pyramid_map.GetAltitudeVarSafe(loc, 50, 0), 0.f);
  EXPECT_EQ(pyramid_map.GetCountSafe(loc, 50, 0), 1);

  loc[0] = 1.0;
  loc[1] = 1.0;
  loc[2] = 1.0;
  EXPECT_FLOAT_EQ(pyramid_map.GetGroundAltitudeSafe(loc, 50, 0), 0.f);
  EXPECT_EQ(pyramid_map.GetGroundCountSafe(loc, 50, 0), 0);

  // reset the caches and pool
  pyramid_map.InitMapNodeCaches(6, 16);

  if (config != nullptr) {
    delete config;
    config = nullptr;
  }
}

TEST_F(PyramidMapTestSuite, test_init) {
  // init config
  PyramidMapConfig* config = new PyramidMapConfig("lossy_full_alt");
  config->SetMapNodeSize(2, 2);
  config->resolution_num_ = 1;
  config->map_folder_path_ = "test_map";

  // create and save nodes
  unsigned int M = 4;
  unsigned int N = 4;
  std::vector<MapNodeIndex> indexes;
  for (unsigned int m = 0; m < M; ++m) {
    for (unsigned int n = 0; n < N; ++n) {
      // create map node index
      MapNodeIndex index;
      index.resolution_id_ = 0;
      index.zone_id_ = 50;
      index.m_ = m;
      index.n_ = n;
      CreateTestMapNode(m, n, index, config);
      indexes.push_back(index);
    }
  }
  // save config file
  config->Save("test_map/config.xml");

  // init pyramid map pool
  PyramidMapNodePool pm_node_pool(16, 4);
  pm_node_pool.Initial(config);

  // init pyramid map
  PyramidMap pyramid_map(config);
  pyramid_map.InitMapNodeCaches(4, 15);
  pyramid_map.AttachMapNodePool(&pm_node_pool);
  EXPECT_TRUE(pyramid_map.SetMapFolderPath(config->map_folder_path_));

  // check fail case
  double data[] = {0.125 * (config->map_node_size_x_ + 1),
                   0.125 * (config->map_node_size_x_ + 1), 1.0};
  Eigen::Vector3d loc(data);
  EXPECT_TRUE(pyramid_map.LoadMapArea(loc, 0, 50, 0, 0));
}

}  // namespace pyramid_map
}  // namespace msf
}  // namespace localization
}  // namespace apollo
