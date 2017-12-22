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
#include <fstream>
#include <map>
#include <vector>

#include "modules/common/log.h"
#include "modules/common/math/quaternion.h"
#include "modules/localization/msf/common/io/velodyne_utility.h"
#include "modules/localization/msf/local_tool/local_visualization/offline_visual/offline_local_visualizer.h"

int main(int argc, char **argv) {
  boost::program_options::options_description boost_desc("Allowed options");
  boost_desc.add_options()("help", "produce help message")(
      "in_folder",
      boost::program_options::value<std::string>(),
      "provide the output folder")(
      "loc_file_a",
      boost::program_options::value<std::string>(),
      "provide gnss localization file.")(
      "loc_file_b",
      boost::program_options::value<std::string>(),
      "provide lidar localization file.")(
      "compare_file",
      boost::program_options::value<std::string>(),
      "provide compare file.");

  boost::program_options::variables_map boost_args;
  boost::program_options::store(
      boost::program_options::parse_command_line(argc, argv, boost_desc),
      boost_args);
  boost::program_options::notify(boost_args);

  if (boost_args.count("help") ||
      !boost_args.count("in_folder")) {
    std::cout << boost_desc << std::endl;
    return 0;
  }

  const std::string in_folder =
      boost_args["in_folder"].as<std::string>();

  const std::string loc_file_a = in_folder + "/" +
      boost_args["loc_file_a"].as<std::string>();
  const std::string loc_file_b = in_folder + "/" +
      boost_args["loc_file_b"].as<std::string>();
  const std::string compare_file = in_folder + "/" +
      boost_args["compare_file"].as<std::string>();

  std::vector<Eigen::Affine3d> poses_a;
  std::vector<Eigen::Vector3d> stds_a;
  std::vector<double> timestamps_a;
  apollo::localization::msf::velodyne::
      LoadPosesAndStds(loc_file_a, &poses_a, &stds_a, &timestamps_a);

  std::vector<Eigen::Affine3d> poses_b;
  std::vector<Eigen::Vector3d> stds_b;
  std::vector<double> timestamps_b;
  apollo::localization::msf::velodyne::
      LoadPosesAndStds(loc_file_b, &poses_b, &stds_b, &timestamps_b);

  std::map<unsigned int, Eigen::Affine3d> out_poses_a;
  std::map<unsigned int, Eigen::Vector3d> out_stds_a;
  std::map<unsigned int, Eigen::Affine3d> out_poses_b;
  std::map<unsigned int, Eigen::Vector3d> out_stds_b;

  apollo::localization::msf::OfflineLocalVisualizer::
      PoseAndStdInterpolationByTime(poses_b, stds_b,
        timestamps_b, timestamps_a, &out_poses_b, &out_stds_b);
  apollo::localization::msf::OfflineLocalVisualizer::
      PoseAndStdInterpolationByTime(poses_a, stds_a,
        timestamps_a, timestamps_a, &out_poses_a, &out_stds_a);

  std::ofstream file(compare_file.c_str());

  for (unsigned int idx = 0; idx < timestamps_a.size(); idx++) {
    auto pose_a_found_iter = out_poses_a.find(idx);
    if (pose_a_found_iter != out_poses_a.end()) {
      AINFO << "Find pose a.";
      const Eigen::Affine3d &pose_a = pose_a_found_iter->second;

      Eigen::Quaterniond quatd_a(pose_a.linear());
      Eigen::Translation3d transd_a(pose_a.translation());
      apollo::common::math::EulerAnglesZXY<double> euler_a(
          quatd_a.w(), quatd_a.x(), quatd_a.y(), quatd_a.z());
      double roll_a = euler_a.roll();
      double pitch_a = euler_a.pitch();
      double yaw_a = euler_a.yaw();

      auto pose_b_found_iter = out_poses_b.find(idx);
      if (pose_b_found_iter != out_poses_b.end()) {
        AINFO << "Find pose b.";
        const Eigen::Affine3d &pose_b = pose_b_found_iter->second;
        Eigen::Quaterniond quatd_b(pose_b.linear());
        Eigen::Translation3d transd_b(pose_b.translation());
        apollo::common::math::EulerAnglesZXY<double> euler_b(
          quatd_b.w(), quatd_b.x(), quatd_b.y(), quatd_b.z());
        double roll_b = euler_b.roll();
        double pitch_b = euler_b.pitch();
        double yaw_b = euler_b.yaw();

        double x_diff = fabs(transd_a.x() - transd_b.x());
        double y_diff = fabs(transd_a.y() - transd_b.y());
        double z_diff = fabs(transd_a.z() - transd_b.z());
        double roll_diff = fabs(roll_a - roll_b) * 180.0 / 3.1415926;
        double pitch_diff = fabs(pitch_a - pitch_b) * 180.0 / 3.1415926;
        double yaw_diff = fabs(yaw_a - yaw_b) * 180.0 / 3.1415926;

        file << idx << " " << timestamps_a[idx] << " "
          << x_diff << " " << y_diff << " " << z_diff << " "
          << roll_diff << " " << pitch_diff << " " << yaw_diff << std::endl;
      }
    }
  }

  file.close();
}
