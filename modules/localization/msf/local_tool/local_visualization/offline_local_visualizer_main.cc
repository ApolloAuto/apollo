#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include "modules/localization/msf/local_tool/local_visualization/offline_local_visualizer.h"

using namespace apollo::localization::msf;
int main(int argc, char **argv) {
  boost::program_options::options_description boost_desc("Allowed options");
  boost_desc.add_options()("help", "produce help message")(
      "basedir", boost::program_options::value<std::string>(),
      "provide the data base dir");

  boost::program_options::variables_map boost_args;
  boost::program_options::store(
      boost::program_options::parse_command_line(argc, argv, boost_desc),
      boost_args);
  boost::program_options::notify(boost_args);

  if (boost_args.count("help") || !boost_args.count("basedir")) {
    std::cout << boost_desc << std::endl;
    return 0;
  }

  const std::string basedir = boost_args["basedir"].as<std::string>();
  std::string map_folder = basedir + "/local_map";
  std::string pcd_folder = basedir + "/pcd";
  std::string gnss_loc_file = pcd_folder + "/gnss_loc.txt";
  std::string lidar_loc_file = pcd_folder + "/lidar_loc.txt";
  std::string fusion_loc_file = pcd_folder + "/fusion_loc.txt";
  std::string extrinsic_file = basedir + "/velodyne_novatel_extrinsics.yaml";

  OfflineLocalVisualizer local_visualizer;
  bool success =
      local_visualizer.Init(map_folder, pcd_folder, gnss_loc_file,
                            lidar_loc_file, fusion_loc_file, extrinsic_file);
  if (!success) {
    return -1;
  }
  local_visualizer.Visualize();

  return 0;
}
