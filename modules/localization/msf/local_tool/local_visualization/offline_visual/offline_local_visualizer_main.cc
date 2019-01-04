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

#include <boost/program_options.hpp>

#include "modules/localization/msf/local_tool/local_visualization/offline_visual/offline_local_visualizer.h"

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
  std::string map_visual_folder = basedir + "/local_map/map_visual";
  std::string pcd_folder = basedir + "/pcd";
  std::string pcd_timestamp_file = pcd_folder + "/pcd_timestamp.txt";
  std::string gnss_loc_file = pcd_folder + "/gnss_loc.txt";
  std::string lidar_loc_file = pcd_folder + "/lidar_loc.txt";
  std::string fusion_loc_file = pcd_folder + "/fusion_loc.txt";
  std::string extrinsic_file = basedir + "/velodyne_novatel_extrinsics.yaml";

  apollo::localization::msf::OfflineLocalVisualizer local_visualizer;
  bool success = local_visualizer.Init(
      map_folder, map_visual_folder, pcd_folder, pcd_timestamp_file,
      gnss_loc_file, lidar_loc_file, fusion_loc_file, extrinsic_file);
  if (!success) {
    return -1;
  }
  local_visualizer.Visualize();

  return 0;
}
