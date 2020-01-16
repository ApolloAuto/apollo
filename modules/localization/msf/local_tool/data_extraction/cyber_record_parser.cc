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

#include "cyber/cyber.h"
#include "modules/localization/msf/local_tool/data_extraction/cyber_record_reader.h"
#include "modules/localization/msf/local_tool/data_extraction/location_exporter.h"
#include "modules/localization/msf/local_tool/data_extraction/pcd_exporter.h"

using apollo::localization::msf::CyberRecordReader;
using apollo::localization::msf::LocationExporter;
using apollo::localization::msf::PCDExporter;

int main(int argc, char **argv) {
  boost::program_options::options_description boost_desc("Allowed options");
  boost_desc.add_options()("help", "produce help message")(
      "bag_file", boost::program_options::value<std::string>(),
      "provide the bag file")("out_folder",
                              boost::program_options::value<std::string>(),
                              "provide the output folder")(
      "cloud_topic",
      boost::program_options::value<std::string>()->default_value(
          "/apollo/sensor/velodyne64/compensator/PointCloud2"),
      "provide point cloud2 topic")(
      "gnss_loc_topic",
      boost::program_options::value<std::string>()->default_value(
          "/apollo/localization/measure_gnss"),
      "provide gnss localization topic")(
      "lidar_loc_topic",
      boost::program_options::value<std::string>()->default_value(
          "/apollo/localization/measure_lidar"),
      "provide lidar localization topic")(
      "fusion_loc_topic",
      boost::program_options::value<std::string>()->default_value(
          "/apollo/localization/pose"),
      "provide fusion localization topic")(
      "odometry_loc_topic",
      boost::program_options::value<std::string>()->default_value(
          "/apollo/sensor/gnss/odometry"),
      "provide odometry localization topic");

  boost::program_options::variables_map boost_args;
  boost::program_options::store(
      boost::program_options::parse_command_line(argc, argv, boost_desc),
      boost_args);
  boost::program_options::notify(boost_args);

  if (boost_args.count("help") || !boost_args.count("bag_file") ||
      !boost_args.count("out_folder")) {
    AERROR << boost_desc;
    return 0;
  }

  const std::string bag_file = boost_args["bag_file"].as<std::string>();
  const std::string pcd_folder =
      boost_args["out_folder"].as<std::string>() + "/pcd";
  if (!boost::filesystem::exists(pcd_folder)) {
    boost::filesystem::create_directory(pcd_folder);
  }

  const std::string cloud_topic = boost_args["cloud_topic"].as<std::string>();
  const std::string gnss_loc_topic =
      boost_args["gnss_loc_topic"].as<std::string>();
  const std::string lidar_loc_topic =
      boost_args["lidar_loc_topic"].as<std::string>();
  const std::string fusion_loc_topic =
      boost_args["fusion_loc_topic"].as<std::string>();
  const std::string odometry_loc_topic =
      boost_args["odometry_loc_topic"].as<std::string>();

  std::unique_ptr<PCDExporter> pcd_exporter(new PCDExporter(pcd_folder));
  std::unique_ptr<LocationExporter> loc_exporter(
      new LocationExporter(pcd_folder));

  CyberRecordReader reader;
  reader.Subscribe(cloud_topic, [&](const std::string &msg) {
    pcd_exporter->CompensatedPcdCallback(msg);
  });

  reader.Subscribe(gnss_loc_topic, [&](const std::string &msg) {
    loc_exporter->GnssLocCallback(msg);
  });

  reader.Subscribe(lidar_loc_topic, [&](const std::string &msg) {
    loc_exporter->LidarLocCallback(msg);
  });

  reader.Subscribe(fusion_loc_topic, [&](const std::string &msg) {
    loc_exporter->FusionLocCallback(msg);
  });

  reader.Subscribe(odometry_loc_topic, [&](const std::string &msg) {
    loc_exporter->OdometryLocCallback(msg);
  });

  reader.Read(bag_file);

  return 0;
}
