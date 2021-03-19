/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/localization/msf/local_pyramid_map/pyramid_map/pyramid_map.h"
#include "modules/localization/msf/local_pyramid_map/pyramid_map/pyramid_map_config.h"
#include "modules/localization/msf/local_pyramid_map/pyramid_map/pyramid_map_matrix.h"
#include "modules/localization/msf/local_pyramid_map/pyramid_map/pyramid_map_pool.h"

using apollo::localization::msf::pyramid_map::MapNodeIndex;
using apollo::localization::msf::pyramid_map::PyramidMap;
using apollo::localization::msf::pyramid_map::PyramidMapConfig;
using apollo::localization::msf::pyramid_map::PyramidMapMatrix;
using apollo::localization::msf::pyramid_map::PyramidMapNode;
using apollo::localization::msf::pyramid_map::PyramidMapNodePool;

namespace apollo {
namespace localization {
namespace msf {

MapNodeIndex GetMapIndexFromMapFolder(const std::string& map_folder) {
  MapNodeIndex index;
  char buf[100];
  sscanf(map_folder.c_str(), "/%03u/%05s/%02d/%08u/%08u", &index.resolution_id_,
         buf, &index.zone_id_, &index.m_, &index.n_);
  std::string zone = buf;
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

  PyramidMapConfig lossless_config("lossless_map");
  PyramidMapNodePool lossless_map_node_pool(25, 8);
  lossless_map_node_pool.Initial(&lossless_config);

  PyramidMap lossless_map(&lossless_config);
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
  apollo::localization::msf::GetAllMapIndex(src_map_folder, dst_map_folder,
                                            &buf);
  AINFO << "index size: " << buf.size();

  PyramidMapConfig config_transform_lossy("lossless_map");
  config_transform_lossy.Load(src_map_folder + "config.xml");
  config_transform_lossy.map_version_ = "lossy_map";
  config_transform_lossy.Save(dst_map_folder + "config.xml");

  AINFO << "lossy map directory structure has built.";

  PyramidMapNodePool lossy_map_node_pool(25, 8);
  lossy_map_node_pool.Initial(&config_transform_lossy);
  PyramidMap lossy_map(&config_transform_lossy);
  lossy_map.InitMapNodeCaches(12, 24);
  lossy_map.AttachMapNodePool(&lossy_map_node_pool);
  if (!lossy_map.SetMapFolderPath(dst_map_folder)) {
    AINFO << "lossy_map config xml not exist";
  }

  int index = 0;
  auto itr = buf.begin();
  for (; itr != buf.end(); ++itr, ++index) {
    PyramidMapNode* lossless_node =
        static_cast<PyramidMapNode*>(lossless_map.GetMapNodeSafe(*itr));
    if (lossless_node == nullptr) {
      AWARN << "index: " << index << " is a nullptr pointer!";
      continue;
    }
    PyramidMapMatrix& lossless_matrix =
        static_cast<PyramidMapMatrix&>(lossless_node->GetMapCellMatrix());

    PyramidMapNode* lossy_node =
        static_cast<PyramidMapNode*>(lossy_map.GetMapNodeSafe(*itr));
    PyramidMapMatrix& lossy_matrix =
        static_cast<PyramidMapMatrix&>(lossy_node->GetMapCellMatrix());

    int rows = lossless_config.map_node_size_y_;
    int cols = lossless_config.map_node_size_x_;
    for (int row = 0; row < rows; ++row) {
      for (int col = 0; col < cols; ++col) {
        const float* intensity = lossless_matrix.GetIntensitySafe(row, col);
        const float* intensity_var =
            lossless_matrix.GetIntensityVarSafe(row, col);
        const unsigned int* count = lossless_matrix.GetCountSafe(row, col);
        // Read altitude
        const float* altitude_avg = lossless_matrix.GetAltitudeSafe(row, col);
        const float* altitude_var =
            lossless_matrix.GetAltitudeVarSafe(row, col);
        const float* altitude_ground =
            lossless_matrix.GetGroundAltitudeSafe(row, col);
        const unsigned int* ground_count =
            lossless_matrix.GetGroundCountSafe(row, col);
        if (intensity) {
          lossy_matrix.SetIntensitySafe(*intensity, row, col);
        }
        if (intensity_var) {
          lossy_matrix.SetIntensityVarSafe(*intensity_var, row, col);
        }
        if (count) {
          lossy_matrix.SetCountSafe(*count, row, col);
        }
        if (altitude_avg) {
          lossy_matrix.SetAltitudeSafe(*altitude_avg, row, col);
        }
        if (altitude_var) {
          lossy_matrix.SetAltitudeVarSafe(*altitude_var, row, col);
        }
        if (altitude_ground) {
          lossy_matrix.SetGroundAltitudeSafe(*altitude_ground, row, col);
        }
        if (ground_count) {
          lossy_matrix.SetGroundCountSafe(*ground_count, row, col);
        }
      }
    }
    lossy_node->SetIsChanged(true);
  }

  return 0;
}
