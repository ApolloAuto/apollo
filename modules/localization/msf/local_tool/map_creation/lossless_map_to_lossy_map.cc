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

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include "cyber/common/log.h"
#include "modules/localization/msf/local_map/lossless_map/lossless_map.h"
#include "modules/localization/msf/local_map/lossless_map/lossless_map_config.h"
#include "modules/localization/msf/local_map/lossless_map/lossless_map_matrix.h"
#include "modules/localization/msf/local_map/lossless_map/lossless_map_pool.h"

#include "modules/localization/msf/local_map/lossy_map/lossy_map_2d.h"
#include "modules/localization/msf/local_map/lossy_map/lossy_map_config_2d.h"
#include "modules/localization/msf/local_map/lossy_map/lossy_map_matrix_2d.h"
#include "modules/localization/msf/local_map/lossy_map/lossy_map_node_2d.h"
#include "modules/localization/msf/local_map/lossy_map/lossy_map_pool_2d.h"

namespace apollo {
namespace localization {
namespace msf {

typedef LossyMap2D LossyMap;
typedef LossyMapNode2D LossyMapNode;
typedef LossyMapNodePool2D LossyMapNodePool;
typedef LossyMapMatrix2D LossyMapMatrix;
typedef LossyMapConfig2D LossyMapConfig;

MapNodeIndex GetMapIndexFromMapFolder(const std::string& map_folder) {
  MapNodeIndex index;
  char buf[100];
  sscanf(map_folder.c_str(), "/%03u/%05s/%02d/%08u/%08u", &index.resolution_id_,
         buf, &index.zone_id_, &index.m_, &index.n_);
  std::string zone = buf;
  // std::cout << zone << std::endl;
  if (zone == "south") {
    index.zone_id_ = -index.zone_id_;
  }
  ADEBUG << index;
  return index;
}

bool GetAllMapIndex(const std::string& src_map_folder,
                    const std::string& dst_map_folder,
                    std::list<MapNodeIndex>* buf) {
  std::string src_map_path = src_map_folder + "/map";
  std::string dst_map_path = dst_map_folder + "/map";
  boost::filesystem::path src_map_path_boost(src_map_path);
  boost::filesystem::path dst_map_path_boost(dst_map_path);

  if (!boost::filesystem::exists(dst_map_path)) {
    boost::filesystem::create_directory(dst_map_path_boost);
  }

  // push path of map's index to list
  buf->clear();
  // std::deque<std::string> map_bin_path;
  boost::filesystem::recursive_directory_iterator end_iter;
  boost::filesystem::recursive_directory_iterator iter(src_map_path_boost);
  for (; iter != end_iter; ++iter) {
    if (!boost::filesystem::is_directory(*iter)) {
      if (iter->path().extension() == "") {
        // map_bin_path.push_back(iter->path().string());
        std::string tmp = iter->path().string();
        tmp = tmp.substr(src_map_path.length(), tmp.length());
        buf->push_back(GetMapIndexFromMapFolder(tmp));
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

}  // namespace msf
}  // namespace localization
}  // namespace apollo

using apollo::localization::msf::LosslessMap;
using apollo::localization::msf::LosslessMapConfig;
using apollo::localization::msf::LosslessMapMatrix;
using apollo::localization::msf::LosslessMapNode;
using apollo::localization::msf::LosslessMapNodePool;
using apollo::localization::msf::LossyMap;
using apollo::localization::msf::LossyMapConfig;
using apollo::localization::msf::LossyMapMatrix;
using apollo::localization::msf::LossyMapNode;
using apollo::localization::msf::LossyMapNodePool;
using apollo::localization::msf::MapNodeIndex;

int main(int argc, char** argv) {
  boost::program_options::options_description boost_desc("Allowed options");
  boost_desc.add_options()("help", "produce help message")(
      "srcdir", boost::program_options::value<std::string>(),
      "provide the data base dir")("dstdir",
                                   boost::program_options::value<std::string>(),
                                   "provide the lossy map destination dir");

  boost::program_options::variables_map boost_args;
  boost::program_options::store(
      boost::program_options::parse_command_line(argc, argv, boost_desc),
      boost_args);
  boost::program_options::notify(boost_args);

  if (boost_args.count("help") || !boost_args.count("srcdir") ||
      !boost_args.count("dstdir")) {
    AERROR << boost_desc;
    return 0;
  }

  const std::string src_path = boost_args["srcdir"].as<std::string>();
  const std::string dst_path = boost_args["dstdir"].as<std::string>();
  std::string src_map_folder = src_path + "/";

  LosslessMapConfig lossless_config("lossless_map");
  LosslessMapNodePool lossless_map_node_pool(25, 8);
  lossless_map_node_pool.Initial(&lossless_config);

  LosslessMap lossless_map(&lossless_config);
  lossless_map.InitMapNodeCaches(12, 24);
  lossless_map.AttachMapNodePool(&lossless_map_node_pool);
  if (!lossless_map.SetMapFolderPath(src_map_folder)) {
    AERROR << "Reflectance map folder is invalid!";
    return -1;
  }

  // create lossy map
  std::string dst_map_folder = dst_path + "/lossy_map/";
  if (!boost::filesystem::exists(dst_map_folder)) {
    boost::filesystem::create_directory(dst_map_folder);
  }

  std::list<MapNodeIndex> buf;
  GetAllMapIndex(src_map_folder, dst_map_folder, &buf);
  AINFO << "index size: " << buf.size();

  LosslessMapConfig config_transform_lossy("lossless_map");
  config_transform_lossy.Load(src_map_folder + "config.xml");
  config_transform_lossy.map_version_ = "lossy_map";
  config_transform_lossy.Save(dst_map_folder + "config.xml");

  AINFO << "lossy map directory structure has built.";

  LossyMapConfig lossy_config("lossy_map");
  LossyMapNodePool lossy_map_node_pool(25, 8);
  lossy_map_node_pool.Initial(&lossy_config);
  LossyMap lossy_map(&lossy_config);
  lossy_map.InitMapNodeCaches(12, 24);
  lossy_map.AttachMapNodePool(&lossy_map_node_pool);
  if (!lossy_map.SetMapFolderPath(dst_map_folder)) {
    AINFO << "lossy_map config xml not exist";
  }

  int index = 0;
  auto itr = buf.begin();
  for (; itr != buf.end(); ++itr, ++index) {
    // int single_alt = 0;
    // int double_alt = 0;
    // float delta_alt_max = 0.0f;
    // float delta_alt_min = 100.0f;
    // int delta_alt_minus_num = 0;
    // float alt_max = 0.0f;
    // float alt_min = 100.0f;

    LosslessMapNode* lossless_node =
        static_cast<LosslessMapNode*>(lossless_map.GetMapNodeSafe(*itr));
    if (lossless_node == nullptr) {
      AWARN << "index: " << index << " is a nullptr pointer!";
      continue;
    }
    LosslessMapMatrix& lossless_matrix =
        static_cast<LosslessMapMatrix&>(lossless_node->GetMapCellMatrix());

    LossyMapNode* lossy_node =
        static_cast<LossyMapNode*>(lossy_map.GetMapNodeSafe(*itr));
    LossyMapMatrix& lossy_matrix =
        static_cast<LossyMapMatrix&>(lossy_node->GetMapCellMatrix());

    int rows = lossless_config.map_node_size_y_;
    int cols = lossless_config.map_node_size_x_;
    for (int row = 0; row < rows; ++row) {
      for (int col = 0; col < cols; ++col) {
        float intensity = lossless_node->GetValue(row, col);
        float intensity_var = lossless_node->GetVar(row, col);
        unsigned int count = lossless_node->GetCount(row, col);

        // Read altitude
        float altitude_ground = 0.0f;
        float altitude_avg = 0.0f;
        bool is_ground_useful = false;
        std::vector<float> layer_alts;
        std::vector<unsigned int> layer_counts;
        lossless_matrix.GetMapCell(row, col).GetCount(&layer_counts);
        lossless_matrix.GetMapCell(row, col).GetAlt(&layer_alts);
        if (layer_counts.empty() || layer_alts.empty()) {
          altitude_avg = lossless_node->GetAlt(row, col);
          is_ground_useful = false;
        } else {
          altitude_avg = lossless_node->GetAlt(row, col);
          altitude_ground = layer_alts[0];
          is_ground_useful = true;
        }

        lossy_matrix[row][col].intensity = intensity;
        lossy_matrix[row][col].intensity_var = intensity_var;
        lossy_matrix[row][col].count = count;
        lossy_matrix[row][col].altitude = altitude_avg;
        lossy_matrix[row][col].altitude_ground = altitude_ground;
        lossy_matrix[row][col].is_ground_useful = is_ground_useful;
      }
    }
    lossy_node->SetIsChanged(true);
  }

  return 0;
}
