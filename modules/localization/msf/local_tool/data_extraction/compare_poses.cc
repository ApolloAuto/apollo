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

#include <fstream>
#include <iomanip>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "yaml-cpp/yaml.h"

#include "cyber/common/log.h"
#include "modules/common/math/quaternion.h"

#include "modules/localization/msf/common/io/velodyne_utility.h"

using ::apollo::common::EigenAffine3dVec;
using ::apollo::common::EigenVector3dVec;

static bool LoadGnssAntennaExtrinsic(const std::string &file_path,
                                     Eigen::Vector3d *imu_ant_offset) {
  CHECK_NOTNULL(imu_ant_offset);

  YAML::Node config = YAML::LoadFile(file_path);
  if (config["leverarm"]) {
    if (config["leverarm"]["primary"]["offset"]) {
      (*imu_ant_offset)[0] =
          config["leverarm"]["primary"]["offset"]["x"].as<double>();
      (*imu_ant_offset)[1] =
          config["leverarm"]["primary"]["offset"]["y"].as<double>();
      (*imu_ant_offset)[2] =
          config["leverarm"]["primary"]["offset"]["z"].as<double>();
    }
    return true;
  }
  return false;
}

static void PoseAndStdInterpolationByTime(
    const EigenAffine3dVec &in_poses, const EigenVector3dVec &in_stds,
    const std::vector<double> &in_timestamps,
    const std::vector<double> &ref_timestamps,
    std::map<unsigned int, Eigen::Affine3d> *out_poses,
    std::map<unsigned int, Eigen::Vector3d> *out_stds) {
  unsigned int index = 0;
  for (size_t i = 0; i < ref_timestamps.size(); ++i) {
    double ref_timestamp = ref_timestamps[i];
    while (index < in_timestamps.size() &&
           in_timestamps.at(index) < ref_timestamp) {
      ++index;
    }

    if (index < in_timestamps.size()) {
      if (index >= 1) {
        double cur_timestamp = in_timestamps[index];
        double pre_timestamp = in_timestamps[index - 1];
        DCHECK_NE(cur_timestamp, pre_timestamp);

        double t =
            (cur_timestamp - ref_timestamp) / (cur_timestamp - pre_timestamp);
        DCHECK_GE(t, 0.0);
        DCHECK_LE(t, 1.0);

        Eigen::Affine3d pre_pose = in_poses[index - 1];
        Eigen::Affine3d cur_pose = in_poses[index];
        Eigen::Quaterniond pre_quatd(pre_pose.linear());
        Eigen::Translation3d pre_transd(pre_pose.translation());
        Eigen::Quaterniond cur_quatd(cur_pose.linear());
        Eigen::Translation3d cur_transd(cur_pose.translation());

        Eigen::Quaterniond res_quatd = pre_quatd.slerp(1 - t, cur_quatd);

        Eigen::Translation3d re_transd;
        re_transd.x() = pre_transd.x() * t + cur_transd.x() * (1 - t);
        re_transd.y() = pre_transd.y() * t + cur_transd.y() * (1 - t);
        re_transd.z() = pre_transd.z() * t + cur_transd.z() * (1 - t);
        (*out_poses)[static_cast<unsigned int>(i)] = re_transd * res_quatd;

        Eigen::Vector3d pre_std = in_stds[index - 1];
        Eigen::Vector3d cur_std = in_stds[index];
        Eigen::Vector3d std;
        std[0] = pre_std[0] * t + cur_std[0] * (1 - t);
        std[1] = pre_std[1] * t + cur_std[1] * (1 - t);
        std[2] = pre_std[2] * t + cur_std[2] * (1 - t);
        (*out_stds)[static_cast<unsigned int>(i)] = std;
      }
    } else {
      AERROR << "[ERROR] No more poses. Exit now.";
      break;
    }
  }
}

int main(int argc, char **argv) {
  boost::program_options::options_description boost_desc("Allowed options");
  boost_desc.add_options()("help", "produce help message")(
      "in_folder", boost::program_options::value<std::string>(),
      "provide the output folder")("loc_file_a",
                                   boost::program_options::value<std::string>(),
                                   "provide gnss localization file.")(
      "loc_file_b", boost::program_options::value<std::string>(),
      "provide lidar localization file.")(
      "compare_file", boost::program_options::value<std::string>(),
      "provide compare file.")(
      "imu_to_ant_offset_file",
      boost::program_options::value<std::string>()->default_value(""),
      "provide imu to ant offset file.");

  boost::program_options::variables_map boost_args;
  boost::program_options::store(
      boost::program_options::parse_command_line(argc, argv, boost_desc),
      boost_args);
  boost::program_options::notify(boost_args);

  if (boost_args.count("help") || !boost_args.count("in_folder")) {
    AERROR << boost_desc << std::endl;
    return 0;
  }

  const std::string in_folder = boost_args["in_folder"].as<std::string>();

  const std::string loc_file_a =
      in_folder + "/" + boost_args["loc_file_a"].as<std::string>();
  const std::string loc_file_b =
      in_folder + "/" + boost_args["loc_file_b"].as<std::string>();
  const std::string compare_file =
      in_folder + "/" + boost_args["compare_file"].as<std::string>();
  const std::string imu_to_ant_offset_file =
      boost_args["imu_to_ant_offset_file"].as<std::string>();

  Eigen::Vector3d imu_ant_offset = Eigen::Vector3d::Zero();
  if (imu_to_ant_offset_file != "") {
    if (!LoadGnssAntennaExtrinsic(imu_to_ant_offset_file, &imu_ant_offset)) {
      return 0;
    }
  }

  EigenAffine3dVec poses_a;
  EigenVector3dVec stds_a;
  std::vector<double> timestamps_a;
  apollo::localization::msf::velodyne::LoadPosesAndStds(loc_file_a, &poses_a,
                                                        &stds_a, &timestamps_a);
  if (poses_a.empty()) {
    return 0;
  }

  EigenAffine3dVec poses_b;
  EigenVector3dVec stds_b;
  std::vector<double> timestamps_b;
  apollo::localization::msf::velodyne::LoadPosesAndStds(loc_file_b, &poses_b,
                                                        &stds_b, &timestamps_b);
  if (poses_b.empty()) {
    return 0;
  }

  std::map<unsigned int, Eigen::Affine3d> out_poses_a;
  std::map<unsigned int, Eigen::Vector3d> out_stds_a;
  std::map<unsigned int, Eigen::Affine3d> out_poses_b;
  std::map<unsigned int, Eigen::Vector3d> out_stds_b;

  PoseAndStdInterpolationByTime(poses_b, stds_b, timestamps_b, timestamps_a,
                                &out_poses_b, &out_stds_b);

  PoseAndStdInterpolationByTime(poses_a, stds_a, timestamps_a, timestamps_a,
                                &out_poses_a, &out_stds_a);

  if (out_poses_a.empty() || out_poses_b.empty()) {
    return 0;
  }

  std::vector<unsigned int> vec_idx;
  std::vector<double> vec_timestamp;
  std::vector<double> vec_x_diff;
  std::vector<double> vec_y_diff;
  std::vector<double> vec_z_diff;
  std::vector<double> vec_roll_diff;
  std::vector<double> vec_pitch_diff;
  std::vector<double> vec_yaw_diff;

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

        Eigen::Vector3d offset = quatd_b * imu_ant_offset;
        transd_a.x() = transd_a.x() - offset[0];
        transd_a.y() = transd_a.y() - offset[1];
        transd_a.z() = transd_a.z() - offset[2];

        double x_diff = fabs(transd_a.x() - transd_b.x());
        double y_diff = fabs(transd_a.y() - transd_b.y());
        double z_diff = fabs(transd_a.z() - transd_b.z());
        double roll_diff = fabs(roll_a - roll_b) * 180.0 / M_PI;
        double pitch_diff = fabs(pitch_a - pitch_b) * 180.0 / M_PI;
        double yaw_diff = fabs(yaw_a - yaw_b);
        yaw_diff = std::min(yaw_diff, 2.0 * M_PI - yaw_diff) * 180.0 / M_PI;

        vec_idx.push_back(idx);
        vec_timestamp.push_back(timestamps_a[idx]);
        vec_x_diff.push_back(x_diff);
        vec_y_diff.push_back(y_diff);
        vec_z_diff.push_back(z_diff);
        vec_roll_diff.push_back(roll_diff);
        vec_pitch_diff.push_back(pitch_diff);
        vec_yaw_diff.push_back(yaw_diff);
      }
    }
  }

  if (vec_idx.empty()) {
    return 0;
  }

  std::ofstream file(compare_file.c_str());
  for (unsigned int i = 0; i < vec_idx.size(); ++i) {
    file << vec_idx[i] << " " << std::setprecision(13) << vec_timestamp[i]
         << " " << vec_x_diff[i] << " " << vec_y_diff[i] << " " << vec_z_diff[i]
         << " " << vec_roll_diff[i] << " " << vec_pitch_diff[i] << " "
         << vec_yaw_diff[i] << std::endl;
  }
  file.close();
}
