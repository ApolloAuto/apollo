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

#include "cyber/common/log.h"
#include "modules/localization/msf/local_tool/map_creation/poses_interpolation/poses_interpolation.h"

int main(int argc, char **argv) {
  boost::program_options::options_description boost_desc("Allowed options");
  boost_desc.add_options()("help", "produce help message")(
      "input_poses_path", boost::program_options::value<std::string>(),
      "provide input poses path")("ref_timestamps_path",
                                  boost::program_options::value<std::string>(),
                                  "provide reference timestamp path")(
      "extrinsic_path", boost::program_options::value<std::string>(),
      "provide velodyne extrinsic path")(
      "output_poses_path", boost::program_options::value<std::string>(),
      "provide output poses path");

  boost::program_options::variables_map boost_args;
  boost::program_options::store(
      boost::program_options::parse_command_line(argc, argv, boost_desc),
      boost_args);
  boost::program_options::notify(boost_args);

  if (boost_args.count("help") || !boost_args.count("input_poses_path") ||
      !boost_args.count("ref_timestamps_path") ||
      !boost_args.count("extrinsic_path") ||
      !boost_args.count("output_poses_path")) {
    AERROR << boost_desc;
    return 0;
  }

  std::string input_poses_path =
      boost_args["input_poses_path"].as<std::string>();
  std::string ref_timestamps_path =
      boost_args["ref_timestamps_path"].as<std::string>();
  std::string extrinsic_path = boost_args["extrinsic_path"].as<std::string>();
  std::string out_poses_path =
      boost_args["output_poses_path"].as<std::string>();

  apollo::localization::msf::PosesInterpolation pose_interpolation;
  bool success = pose_interpolation.Init(input_poses_path, ref_timestamps_path,
                                         out_poses_path, extrinsic_path);

  if (success) {
    pose_interpolation.DoInterpolation();
  }

  return 0;
}
