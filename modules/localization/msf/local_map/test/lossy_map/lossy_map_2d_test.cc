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

#include "modules/localization/msf/local_map/lossy_map/lossy_map_2d.h"
#include <gtest/gtest.h>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include "modules/localization/msf/local_map/lossy_map/lossy_map_matrix_2d.h"
#include "modules/localization/msf/local_map/lossy_map/lossy_map_node_2d.h"
#include "modules/localization/msf/local_map/lossy_map/lossy_map_pool_2d.h"

namespace apollo {
namespace localization {
namespace msf {

MapNodeIndex GetMapIndexFromMapFolder(const std::string& map_folder) {
  MapNodeIndex index;
  char buf[100];
  sscanf(map_folder.c_str(), "/%03u/%05s/%02d/%08u/%08u", &index.resolution_id_,
         buf, &index.zone_id_, &index.m_, &index.n_);
  std::string zone = buf;
  std::cout << zone << std::endl;
  if (zone == "south") {
    index.zone_id_ = -index.zone_id_;
  }
  std::cout << index << std::endl;
  return index;
}

bool GetAllMapIndex(const std::string& src_map_folder,
                    const std::string& dst_map_folder,
                    std::list<MapNodeIndex>& buf) {
  std::string src_map_path = src_map_folder + "/map";
  std::string dst_map_path = dst_map_folder + "/map";
  boost::filesystem::path src_map_path_boost(src_map_path);
  boost::filesystem::path dst_map_path_boost(dst_map_path);

  if (!boost::filesystem::exists(dst_map_path)) {
    boost::filesystem::create_directory(dst_map_path_boost);
  }

  buf.clear();

  boost::filesystem::recursive_directory_iterator end_iter;
  boost::filesystem::recursive_directory_iterator iter(src_map_path);
  for (; iter != end_iter; ++iter) {
    if (!boost::filesystem::is_directory(*iter)) {
      if (iter->path().extension() == "") {
        std::string tmp = iter->path().string();
        tmp = tmp.substr(src_map_path.length(), tmp.length());
        buf.push_back(GetMapIndexFromMapFolder(tmp));
      }
    } else {
      std::string tmp = iter->path().string();
      tmp = tmp.substr(src_map_path.length(), tmp.length());
      tmp = dst_map_path + tmp;
      boost::filesystem::path p(tmp);
      if (!boost::filesystem::exists(p)) {
        boost::filesystem::create_directory(p);
      }
    }
  }

  return true;
}

class LossyMap2DTestSuite : public ::testing::Test {
 protected:
  LossyMap2DTestSuite() {}
  virtual ~LossyMap2DTestSuite() {}
  virtual void SetUp() {}
  virtual void TearDown() {}
};

TEST_F(LossyMap2DTestSuite, LossyMapToMapMatrixTest) {
  typedef LossyMap2D LossyMap;
  typedef LossyMapNode2D LossyMapNode;
  typedef LossyMapNodePool2D LossyMapNodePool;
  typedef LossyMapMatrix2D LossyMapMatrix;
  typedef LossyMapConfig2D LossyMapConfig;

  std::string src_map_folder =
      "modules/localization/msf/local_map/test/test_data/lossy_single_map";
  std::string dst_map_folder =
      "modules/localization/msf/local_map/test/test_data/temp_output_lossy_map";
  if (!boost::filesystem::exists(dst_map_folder)) {
    boost::filesystem::create_directory(dst_map_folder);
  }

  LossyMapConfig input_config("lossy_map");

  LossyMapNodePool input_node_pool(25, 8);
  input_node_pool.Initial(&input_config);
  LossyMap input_map(input_config);
  input_map.InitThreadPool(1, 6);
  input_map.InitMapNodeCaches(12, 24);
  input_map.AttachMapNodePool(&input_node_pool);
  if (!input_map.SetMapFolderPath(src_map_folder)) {
    std::cerr << "Reflectance map folder is invalid!" << std::endl;
    return;
  }

  std::list<MapNodeIndex> buf;
  GetAllMapIndex(src_map_folder, dst_map_folder, buf);
  std::cout << "index size: " << buf.size() << std::endl;

  input_config.Save(dst_map_folder + "/config.xml");

  LossyMapConfig lossy_config("lossy_map");
  LossyMapNodePool lossy_map_node_pool(25, 8);
  lossy_map_node_pool.Initial(&lossy_config);

  LossyMap lossy_map(lossy_config);
  lossy_map.InitThreadPool(1, 6);
  lossy_map.InitMapNodeCaches(12, 24);
  lossy_map.AttachMapNodePool(&lossy_map_node_pool);
  if (!lossy_map.SetMapFolderPath(dst_map_folder)) {
    std::cout << "lossy_map config xml not exist" << std::endl;
  }

  int index = 0;
  auto itr = buf.begin();
  for (; itr != buf.end(); ++itr, ++index) {
    LossyMapNode* node =
        static_cast<LossyMapNode*>(input_map.GetMapNodeSafe(*itr));
    if (node == NULL) {
      std::cerr << "index: " << index << " is a NULL pointer!" << std::endl;
      continue;
    }
    LossyMapMatrix& input_matrix =
        static_cast<LossyMapMatrix&>(node->GetMapCellMatrix());

    LossyMapNode* lossy_node =
        static_cast<LossyMapNode*>(lossy_map.GetMapNodeSafe(*itr));
    LossyMapMatrix& lossy_matrix =
        static_cast<LossyMapMatrix&>(lossy_node->GetMapCellMatrix());

    int rows = input_config.map_node_size_y_;
    int cols = input_config.map_node_size_x_;
    for (int row = 0; row < rows; ++row) {
      for (int col = 0; col < cols; ++col) {
        lossy_matrix[row][col].intensity = input_matrix[row][col].intensity;
        lossy_matrix[row][col].intensity_var =
            input_matrix[row][col].intensity_var;
        lossy_matrix[row][col].count = input_matrix[row][col].count;
        lossy_matrix[row][col].altitude = input_matrix[row][col].altitude;
        lossy_matrix[row][col].altitude_ground =
            input_matrix[row][col].altitude_ground;
        lossy_matrix[row][col].is_ground_useful =
            input_matrix[row][col].is_ground_useful;
      }
    }
    lossy_node->SetIsChanged(true);
  }
}

TEST_F(LossyMap2DTestSuite, MapScheduleTest) {
  typedef LossyMap2D LossyMap;
  typedef LossyMapNodePool2D LossyMapNodePool;
  typedef LossyMapConfig2D LossyMapConfig;

  std::string map_folder =
      "modules/localization/msf/local_map/test/test_data/lossy_single_map";
  LossyMapConfig map_config("lossy_map");

  LossyMapNodePool input_node_pool(25, 8);
  input_node_pool.Initial(&map_config);
  LossyMap lossy_map(map_config);
  lossy_map.InitThreadPool(1, 6);
  lossy_map.InitMapNodeCaches(12, 24);
  lossy_map.AttachMapNodePool(&input_node_pool);
  if (!lossy_map.SetMapFolderPath(map_folder)) {
    std::cerr << "Reflectance map folder is invalid!" << std::endl;
    return;
  }

  unsigned int zone_id = 50;
  unsigned int resolution_id = 0;

  MapNodeIndex index;
  index.m_ = 34636;
  index.n_ = 3436;
  {
    Eigen::Vector3d trans_diff;
    trans_diff[0] = 10;
    trans_diff[1] = 10;

    auto loc = BaseMapNode::GetLeftTopCorner(lossy_map.GetConfig(), index);
    Eigen::Vector3d location;
    location[0] = loc[0];
    location[1] = loc[1];

    for (int i = 0; i < 100; ++i) {
      lossy_map.LoadMapArea(location, resolution_id, zone_id, 21, 21);
      lossy_map.PreloadMapArea(location, trans_diff, resolution_id, zone_id);
      location += trans_diff;
    }
  }
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
