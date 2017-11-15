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

#include "modules/localization/msf/local_map/lossless_map/lossless_map.h"
#include <gtest/gtest.h>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include "modules/localization/msf/local_map/lossless_map/lossless_map_config.h"
#include "modules/localization/msf/local_map/lossless_map/lossless_map_matrix.h"
#include "modules/localization/msf/local_map/lossless_map/lossless_map_node.h"
#include "modules/localization/msf/local_map/lossless_map/lossless_map_pool.h"

namespace apollo {
namespace localization {
namespace msf {

class LosslessMapTestSuite : public ::testing::Test {
 protected:
  LosslessMapTestSuite() {}
  virtual ~LosslessMapTestSuite() {}
  virtual void SetUp() {}
  virtual void TearDown() {}
};

/**@brief Test the binary I/O of the map node. */
TEST_F(LosslessMapTestSuite, MapNodeTest) {
  // Delete map folder
  boost::filesystem::remove_all(
      "modules/localization/msf/local_map/test/testdata/temp_map");

  // Map save
  {
    std::string map_base_folder =
        "modules/localization/msf/local_map/test/testdata";
    LosslessMapConfig config;
    LosslessMap map(config);
    LosslessMapNodePool lossless_map_node_pool(25, 8);
    lossless_map_node_pool.Initial(&config);
    map.InitThreadPool(1, 6);
    map.InitMapNodeCaches(12, 24);
    map.AttachMapNodePool(&lossless_map_node_pool);

    std::string map_folder_path = map_base_folder + "/temp_map";
    if (!boost::filesystem::exists(map_folder_path)) {
      boost::filesystem::create_directory(map_folder_path);
    }
    map.SetMapFolderPath(map_folder_path);

    Eigen::Vector3d pt3d;
    pt3d << 439731.0, 4433988.03, 47.0;
    map.SetValue(pt3d, 50, 60);
    map.SetValue(pt3d, 50, 70);
  }

  // Map load
  {
    std::string map_base_folder =
        "modules/localization/msf/local_map/test/testdata";

    LosslessMapConfig config;
    LosslessMap map(config);
    LosslessMapNodePool lossless_map_node_pool(25, 8);
    lossless_map_node_pool.Initial(&config);
    map.InitThreadPool(1, 6);
    map.InitMapNodeCaches(12, 24);
    map.AttachMapNodePool(&lossless_map_node_pool);
    std::string map_folder_path = map_base_folder + "/temp_map";
    if (!boost::filesystem::exists(map_folder_path)) {
      boost::filesystem::create_directory(map_folder_path);
    }
    map.SetMapFolderPath(map_folder_path);
    {
      Eigen::Vector3d pt3d;
      pt3d << 439731.0, 4433988.03, 47.0;
      ASSERT_EQ(map.GetCountSafe(pt3d, 50, 0), 2);
      ASSERT_EQ(map.GetValueSafe(pt3d, 50, 0), 65);
      ASSERT_LT(fabs(map.GetAltSafe(pt3d, 50, 0) - 47.0), 1e-5);
    }
  }
}

TEST_F(LosslessMapTestSuite, MapScheduleTest) {
  std::string map_folder =
      "modules/localization/msf/local_map/test/testdata/lossless_single_map";
  LosslessMapConfig map_config("lossless_map");

  LosslessMapNodePool input_node_pool(25, 8);
  input_node_pool.Initial(&map_config);
  LosslessMap map(map_config);
  map.InitThreadPool(1, 6);
  map.InitMapNodeCaches(12, 24);
  map.AttachMapNodePool(&input_node_pool);
  if (!map.SetMapFolderPath(map_folder)) {
    std::cerr << "Reflectance map folder is invalid!" << std::endl;
    return;
  }

  int zone_id = 50;
  unsigned int resolution_id = 0;

  Eigen::Vector3d trans_diff;
  trans_diff[0] = 10;
  trans_diff[1] = 10;

  MapNodeIndex index;
  index.m_ = 34635;
  index.n_ = 3437;

  auto loc = LosslessMapNode::GetLeftTopCorner(map.GetConfig(), index);
  Eigen::Vector3d location;
  location[0] = loc[0];
  location[1] = loc[1];

  for (int i = 0; i < 100; ++i) {
    map.LoadMapArea(location, resolution_id, zone_id, 21, 21);
    map.PreloadMapArea(location, trans_diff, resolution_id, zone_id);
    location += trans_diff;
  }

  LosslessMapNode* node =
      static_cast<LosslessMapNode*>(map.GetMapNodeSafe(index));
  node->SetLeftTopCorner(0.0, 0.0);

  std::vector<Eigen::Vector3d> pt3ds_global(200);
  for (float i = 0; i < 200; ++i) {
    pt3ds_global[i][0] = i * 8;
    pt3ds_global[i][1] = i * 5;
    pt3ds_global[i][2] = 20.0 + i / 200.0;
  }

  for (size_t i = 0; i < pt3ds_global.size(); ++i) {
    const Eigen::Vector3d& pt3d = pt3ds_global[i];
    const unsigned char& intensity = 10 + i % 128;
    node->SetValueIfInBound(pt3d, intensity);
    node->SetIsChanged(false); // avoid changing origin data
  }
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
