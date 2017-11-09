#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
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

MapNodeIndex get_map_index_from_map_folder(const std::string& map_folder) {
  MapNodeIndex index;
  char buf[100];
  sscanf(map_folder.c_str(), "/%03u/%05s/%02d/%08u/%08u", &index._resolution_id,
         buf, &index._zone_id, &index._m, &index._n);
  std::string zone = buf;
  // std::cout << zone << std::endl;
  if (zone == "south") {
    index._zone_id = -index._zone_id;
  }
  std::cout << index << std::endl;
  return index;
}

bool get_all_map_index(const std::string& src_map_folder,
                       const std::string& dst_map_folder,
                       std::list<MapNodeIndex>& buf) {
  std::string src_map_path = src_map_folder + "/map";
  std::string dst_map_path = dst_map_folder + "/map";
  boost::filesystem::path src_map_path_boost(src_map_path);
  boost::filesystem::path dst_map_path_boost(dst_map_path);

  if (!boost::filesystem::exists(dst_map_path)) {
    boost::filesystem::create_directory(dst_map_path_boost);
  }

  // push path of map's index to list
  buf.clear();
  // std::deque<std::string> map_bin_path;
  int index = 0;
  boost::filesystem::recursive_directory_iterator end_iter;
  boost::filesystem::recursive_directory_iterator iter(src_map_path);
  for (; iter != end_iter; ++iter) {
    if (!boost::filesystem::is_directory(*iter)) {
      if (iter->path().extension() == "") {
        // map_bin_path.push_back(iter->path().string());
        std::string tmp = iter->path().string();
        tmp = tmp.substr(src_map_path.length(), tmp.length());
        buf.push_back(get_map_index_from_map_folder(tmp));
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

using namespace apollo::localization::msf;

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
    std::cout << boost_desc << std::endl;
    return 0;
  }

  const std::string src_path = boost_args["srcdir"].as<std::string>();
  const std::string dst_path = boost_args["dstdir"].as<std::string>();
  std::string src_map_folder = src_path + "/";

  LosslessMapConfig lossless_config("lossless_map");
  LosslessMapNodePool lossless_map_node_pool(25, 8);
  lossless_map_node_pool.initial(&lossless_config);

  LosslessMap lossless_map(lossless_config);
  lossless_map.init_thread_pool(1, 6);
  lossless_map.init_map_node_caches(12, 24);
  lossless_map.attach_map_node_pool(&lossless_map_node_pool);
  if (!lossless_map.set_map_folder_path(src_map_folder)) {
    std::cerr << "Reflectance map folder is invalid!" << std::endl;
    return -1;
  }

  // create lossy map
  std::string dst_map_folder = dst_path + "/lossy_single_map/";
  if (!boost::filesystem::exists(dst_map_folder)) {
    boost::filesystem::create_directory(dst_map_folder);
  }

  std::list<MapNodeIndex> buf;
  get_all_map_index(src_map_folder, dst_map_folder, buf);
  std::cout << "index size: " << buf.size() << std::endl;

  LosslessMapConfig config_transform_lossy("lossless_map");
  config_transform_lossy.load(src_map_folder + "config.xml");
  config_transform_lossy._map_version = "lossy_full_alt";
  config_transform_lossy.save(dst_map_folder + "config.xml");

  std::cout << "lossy map directory structure has built." << std::endl;

  LossyMapConfig lossy_config("lossy_full_alt");
  LossyMapNodePool lossy_map_node_pool(25, 8);
  lossy_map_node_pool.initial(&lossy_config);
  LossyMap lossy_map(lossy_config);
  lossy_map.init_thread_pool(1, 6);
  lossy_map.init_map_node_caches(12, 24);
  lossy_map.attach_map_node_pool(&lossy_map_node_pool);
  if (!lossy_map.set_map_folder_path(dst_map_folder)) {
    std::cout << "lossy_map config xml not exist" << std::endl;
  }

  int index = 0;
  auto itr = buf.begin();
  for (; itr != buf.end(); ++itr, ++index) {
    // int single_alt = 0;
    // int double_alt = 0;
    // float delta_alt_max = 0.0;
    // float delta_alt_min = 100.0;
    // int delta_alt_minus_num = 0;
    // float alt_max = 0.0;
    // float alt_min = 100.0;

    LosslessMapNode* lossless_node =
        static_cast<LosslessMapNode*>(lossless_map.get_map_node_safe(*itr));
    if (lossless_node == NULL) {
      std::cerr << "index: " << index << " is a NULL pointer!" << std::endl;
      continue;
    }
    LosslessMapMatrix& lossless_matrix =
        static_cast<LosslessMapMatrix&>(lossless_node->get_map_cell_matrix());

    LossyMapNode* lossy_node =
        static_cast<LossyMapNode*>(lossy_map.get_map_node_safe(*itr));
    LossyMapMatrix& lossy_matrix =
        static_cast<LossyMapMatrix&>(lossy_node->get_map_cell_matrix());

    int rows = lossless_config._map_node_size_y;
    int cols = lossless_config._map_node_size_x;
    for (int row = 0; row < rows; ++row) {
      for (int col = 0; col < cols; ++col) {
        float intensity = lossless_node->get_value(row, col);
        float intensity_var = lossless_node->get_var(row, col);
        unsigned int count = lossless_node->get_count(row, col);

        // Read altitude
        float altitude_ground = 0.0;
        float altitude_avg = 0.0;
        bool is_ground_useful = false;
        std::vector<float> layer_alts;
        std::vector<unsigned int> layer_counts;
        lossless_matrix.get_map_cell(row, col).get_count(layer_counts);
        lossless_matrix.get_map_cell(row, col).get_alt(layer_alts);
        if (layer_counts.size() == 0 || layer_alts.size() == 0) {
          altitude_avg = lossless_node->get_alt(row, col);
          is_ground_useful = false;
        } else {
          altitude_avg = lossless_node->get_alt(row, col);
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
    lossy_node->set_is_changed(true);
  }

  return 0;
}
