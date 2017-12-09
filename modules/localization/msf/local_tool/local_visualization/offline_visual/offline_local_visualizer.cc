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

#include "modules/localization/msf/local_tool/local_visualization/offline_visual/offline_local_visualizer.h"
#include <boost/filesystem.hpp>
#include <map>
#include <vector>
#include "modules/localization/msf/common/io/velodyne_utility.h"

namespace apollo {
namespace localization {
namespace msf {
OfflineLocalVisualizer::OfflineLocalVisualizer()
    : map_config_(), resolution_id_(0), zone_id_(0), visual_engine_() {
  map_folder_ = "";
  pcd_folder_ = "";
  gnss_loc_file_ = "";
  lidar_loc_file_ = "";
  fusion_loc_file_ = "";
  extrinsic_file_ = "";
}

OfflineLocalVisualizer::~OfflineLocalVisualizer() {}

bool OfflineLocalVisualizer::Init(const std::string &map_folder,
                                  const std::string &pcd_folder,
                                  const std::string &pcd_timestamp_file,
                                  const std::string &gnss_loc_file,
                                  const std::string &lidar_loc_file,
                                  const std::string &fusion_loc_file,
                                  const std::string &extrinsic_file) {
  map_folder_ = map_folder;
  pcd_folder_ = pcd_folder;
  pcd_timestamp_file_ = pcd_timestamp_file;
  gnss_loc_file_ = gnss_loc_file;
  lidar_loc_file_ = lidar_loc_file;
  fusion_loc_file_ = fusion_loc_file;
  extrinsic_file_ = extrinsic_file;

  pcd_timestamps_.clear();
  gnss_poses_.clear();
  lidar_poses_.clear();
  fusion_poses_.clear();
  gnss_stds_.clear();
  lidar_stds_.clear();
  fusion_stds_.clear();

  std::string config_file = map_folder_ + "/config.xml";
  map_config_.map_version_ = "lossy_map";
  bool success = map_config_.Load(config_file);
  if (!success) {
    std::cerr << "Load map config failed." << std::endl;
    return false;
  }
  std::cout << "Load map config succeed." << std::endl;

  success = velodyne::LoadExtrinsic(extrinsic_file_, &velodyne_extrinsic_);
  if (!success) {
    std::cerr << "Load velodyne extrinsic failed." << std::endl;
    return false;
  }
  std::cout << "Load velodyne extrinsic succeed." << std::endl;

  success = PCDTimestampFileHandler();
  if (!success) {
    std::cerr << "Handle pcd timestamp file failed." << std::endl;
    return false;
  }
  std::cout << "Handle pcd timestamp file succeed." << std::endl;

  success = LidarLocFileHandler(pcd_timestamps_);
  if (!success) {
    std::cerr << "Handle lidar localization file failed." << std::endl;
    return false;
  }
  std::cout << "Handle lidar localization file succeed." << std::endl;

  success = GnssLocFileHandler(pcd_timestamps_);
  if (!success) {
    std::cerr << "Handle gnss localization file failed." << std::endl;
    return false;
  }
  std::cout << "Handle gnss localization file succeed." << std::endl;

  success = FusionLocFileHandler(pcd_timestamps_);
  if (!success) {
    std::cerr << "Handle fusion localization file failed." << std::endl;
    return false;
  }
  std::cout << "Handle fusion localization file succeed." << std::endl;

  resolution_id_ = 0;
  success = GetZoneIdFromMapFolder(map_folder_, resolution_id_, &zone_id_);
  if (!success) {
    std::cerr << "Get zone id failed." << std::endl;
    return false;
  }
  std::cout << "Get zone id succeed." << std::endl;

  success = visual_engine_.Init(map_folder_, map_config_, resolution_id_,
                                zone_id_, velodyne_extrinsic_, LOC_INFO_NUM);
  if (!success) {
    std::cerr << "Visualization engine init failed." << std::endl;
    return false;
  }
  std::cout << "Visualization engine init succeed." << std::endl;

  return true;
}

void OfflineLocalVisualizer::Visualize() {
  for (unsigned int idx = 0; idx < pcd_timestamps_.size(); idx++) {
    LocalizatonInfo lidar_loc_info;
    LocalizatonInfo gnss_loc_info;
    LocalizatonInfo fusion_loc_info;

    std::cout << "Frame id: " << idx + 1 << std::endl;
    auto pose_found_iter = lidar_poses_.find(idx);
    auto std_found_iter = lidar_stds_.find(idx);
    if (pose_found_iter != lidar_poses_.end() &&
        std_found_iter != lidar_stds_.end()) {
      std::cout << "Find lidar pose." << std::endl;
      const Eigen::Affine3d &lidar_pose = pose_found_iter->second;
      const Eigen::Vector3d &lidar_std = std_found_iter->second;
      // lidar_loc_info.set(lidar_pose, "Lidar.", pcd_timestamps_[idx], idx +
      // 1);
      lidar_loc_info.set(Eigen::Translation3d(lidar_pose.translation()),
                         Eigen::Quaterniond(lidar_pose.linear()), lidar_std,
                         "Lidar.", pcd_timestamps_[idx], idx + 1);
    }

    pose_found_iter = gnss_poses_.find(idx);
    std_found_iter = gnss_stds_.find(idx);
    if (pose_found_iter != gnss_poses_.end() &&
        std_found_iter != gnss_stds_.end()) {
      std::cout << "Find gnss pose." << std::endl;
      const Eigen::Affine3d &gnss_pose = pose_found_iter->second;
      const Eigen::Vector3d &gnss_std = std_found_iter->second;
      // gnss_loc_info.set(gnss_pose, "GNSS.", pcd_timestamps_[idx], idx + 1);
      gnss_loc_info.set(Eigen::Translation3d(gnss_pose.translation()), gnss_std,
                        "GNSS.", pcd_timestamps_[idx], idx + 1);
    }

    pose_found_iter = fusion_poses_.find(idx);
    std_found_iter = fusion_stds_.find(idx);
    if (pose_found_iter != fusion_poses_.end() &&
        std_found_iter != fusion_stds_.end()) {
      std::cout << "Find fusion pose." << std::endl;
      const Eigen::Affine3d &fusion_pose = pose_found_iter->second;
      const Eigen::Vector3d &fusion_std = std_found_iter->second;
      // fusion_loc_info.set(fusion_pose, "Fusion.", pcd_timestamps_[idx],
      //                    idx + 1);
      fusion_loc_info.set(Eigen::Translation3d(fusion_pose.translation()),
                          Eigen::Quaterniond(fusion_pose.linear()), fusion_std,
                          "Fusion.", pcd_timestamps_[idx], idx + 1);
    }

    std::vector<LocalizatonInfo> loc_infos;
    loc_infos.push_back(lidar_loc_info);
    loc_infos.push_back(gnss_loc_info);
    loc_infos.push_back(fusion_loc_info);

    std::string pcd_file_path;
    std::ostringstream ss;
    ss << idx + 1;
    pcd_file_path = pcd_folder_ + "/" + ss.str() + ".pcd";
    // std::cout << "pcd_file_path: " << pcd_file_path << std::endl;
    std::vector<Eigen::Vector3d> pt3ds;
    std::vector<unsigned char> intensities;
    apollo::localization::msf::velodyne::LoadPcds(
        pcd_file_path, idx, lidar_loc_info.pose, &pt3ds, &intensities, false);

    visual_engine_.Visualize(loc_infos, pt3ds);
  }
}

bool OfflineLocalVisualizer::PCDTimestampFileHandler() {
  pcd_timestamps_.clear();

  FILE *file = fopen(pcd_timestamp_file_.c_str(), "r");
  if (file) {
    unsigned int index;
    double timestamp;
    int size = 2;
    while ((size = fscanf(file, "%u %lf\n", &index, &timestamp)) == 2) {
      pcd_timestamps_.push_back(timestamp);
    }
    fclose(file);
  } else {
    std::cerr << "Can't open file to read: " << pcd_timestamp_file_
              << std::endl;
    return false;
  }

  return true;
}

bool OfflineLocalVisualizer::LidarLocFileHandler(
    const std::vector<double> &pcd_timestamps) {
  std::vector<Eigen::Affine3d> poses;
  std::vector<Eigen::Vector3d> stds;
  std::vector<double> timestamps;
  // velodyne::LoadPcdPoses(lidar_loc_file_, poses, pcd_timestamps_);
  velodyne::LoadPosesAndStds(lidar_loc_file_, &poses, &stds, &timestamps);

  PoseAndStdInterpolationByTime(poses, stds, timestamps, pcd_timestamps,
                                &lidar_poses_, &lidar_stds_);

  return true;
}

bool OfflineLocalVisualizer::GnssLocFileHandler(
    const std::vector<double> &pcd_timestamps) {
  std::vector<Eigen::Affine3d> poses;
  std::vector<Eigen::Vector3d> stds;
  std::vector<double> timestamps;
  // velodyne::LoadPcdPoses(gnss_loc_file_, poses, timestamps);
  velodyne::LoadPosesAndStds(gnss_loc_file_, &poses, &stds, &timestamps);

  // PoseInterpolationByTime(poses, timestamps, pcd_timestamps, gnss_poses_);
  PoseAndStdInterpolationByTime(poses, stds, timestamps, pcd_timestamps,
                                &gnss_poses_, &gnss_stds_);

  return true;
}

bool OfflineLocalVisualizer::FusionLocFileHandler(
    const std::vector<double> &pcd_timestamps) {
  std::vector<Eigen::Affine3d> poses;
  std::vector<Eigen::Vector3d> stds;
  std::vector<double> timestamps;
  // velodyne::LoadPcdPoses(fusion_loc_file_, poses, timestamps);
  velodyne::LoadPosesAndStds(fusion_loc_file_, &poses, &stds, &timestamps);

  // PoseInterpolationByTime(poses, timestamps, pcd_timestamps, fusion_poses_);
  PoseAndStdInterpolationByTime(poses, stds, timestamps, pcd_timestamps,
                                &fusion_poses_, &fusion_stds_);
  return true;
}

// void OfflineLocalVisualizer::PoseInterpolationByTime(
//     const std::vector<Eigen::Affine3d> &in_poses,
//     const std::vector<double> &in_timestamps,
//     const std::vector<double> &ref_timestamps,
//     std::map<unsigned int, Eigen::Affine3d> &out_poses) {
//   unsigned int index = 0;
//   for (size_t i = 0; i < ref_timestamps.size(); i++) {
//     double ref_timestamp = ref_timestamps[i];
//     // unsigned int ref_frame_id = i;
//     // unsigned int matched_index = 0;
//     while (in_timestamps[index] < ref_timestamp &&
//            index < in_timestamps.size()) {
//       ++index;
//     }

//     if (index < in_timestamps.size()) {
//       if (index >= 1) {
//         double cur_timestamp = in_timestamps[index];
//         double pre_timestamp = in_timestamps[index - 1];
//         assert(cur_timestamp != pre_timestamp);

//         double t =
//             (cur_timestamp - ref_timestamp) / (cur_timestamp -
//             pre_timestamp);
//         assert(t >= 0.0);
//         assert(t <= 1.0);

//         Eigen::Affine3d pre_pose = in_poses[index - 1];
//         Eigen::Affine3d cur_pose = in_poses[index];
//         Eigen::Quaterniond pre_quatd(pre_pose.linear());
//         Eigen::Translation3d pre_transd(pre_pose.translation());
//         Eigen::Quaterniond cur_quatd(cur_pose.linear());
//         Eigen::Translation3d cur_transd(cur_pose.translation());

//         Eigen::Quaterniond res_quatd = pre_quatd.slerp(1 - t, cur_quatd);

//         Eigen::Translation3d re_transd;
//         re_transd.x() = pre_transd.x() * t + cur_transd.x() * (1 - t);
//         re_transd.y() = pre_transd.y() * t + cur_transd.y() * (1 - t);
//         re_transd.z() = pre_transd.z() * t + cur_transd.z() * (1 - t);

//         out_poses[i] = re_transd * res_quatd;
//       }
//     } else {
//       std::cerr << "[ERROR] No more poses. Exit now." << std::endl;
//       break;
//     }
//     // std::cout << "Frame_id: " << i << std::endl;
//   }
// }

void OfflineLocalVisualizer::PoseAndStdInterpolationByTime(
    const std::vector<Eigen::Affine3d> &in_poses,
    const std::vector<Eigen::Vector3d> &in_stds,
    const std::vector<double> &in_timestamps,
    const std::vector<double> &ref_timestamps,
    std::map<unsigned int, Eigen::Affine3d> *out_poses,
    std::map<unsigned int, Eigen::Vector3d> *out_stds) {
  unsigned int index = 0;
  for (size_t i = 0; i < ref_timestamps.size(); i++) {
    double ref_timestamp = ref_timestamps[i];
    // unsigned int ref_frame_id = i;
    // unsigned int matched_index = 0;
    while (in_timestamps[index] < ref_timestamp &&
           index < in_timestamps.size()) {
      ++index;
    }

    if (index < in_timestamps.size()) {
      if (index >= 1) {
        double cur_timestamp = in_timestamps[index];
        double pre_timestamp = in_timestamps[index - 1];
        assert(cur_timestamp != pre_timestamp);

        double t =
            (cur_timestamp - ref_timestamp) / (cur_timestamp - pre_timestamp);
        assert(t >= 0.0);
        assert(t <= 1.0);

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
        (*out_poses)[i] = re_transd * res_quatd;

        Eigen::Vector3d pre_std = in_stds[index - 1];
        Eigen::Vector3d cur_std = in_stds[index];
        Eigen::Vector3d std;
        std[0] = pre_std[0] * t + cur_std[0] * (1 - t);
        std[1] = pre_std[1] * t + cur_std[1] * (1 - t);
        std[2] = pre_std[2] * t + cur_std[2] * (1 - t);
        (*out_stds)[i] = std;
      }
    } else {
      std::cerr << "[ERROR] No more poses. Exit now." << std::endl;
      break;
    }
    // std::cout << "Frame_id: " << i << std::endl;
  }
}

bool OfflineLocalVisualizer::GetZoneIdFromMapFolder(
    const std::string &map_folder, const unsigned int resolution_id,
    int *zone_id) {
  char buf[256];
  snprintf(buf, sizeof(buf), "/%03u", resolution_id);
  std::string folder_north = map_folder + "/map" + buf + "/north";
  std::string folder_south = map_folder + "/map" + buf + "/south";
  boost::filesystem::directory_iterator directory_end;
  boost::filesystem::directory_iterator iter_north(folder_north);
  if (iter_north == directory_end) {
    boost::filesystem::directory_iterator iter_south(folder_south);
    if (iter_south == directory_end) {
      return false;
    } else {
      std::string zone_id_full_path = (*iter_south).path().string();
      std::size_t pos = zone_id_full_path.find_last_of("/");
      std::string zone_id_str =
          zone_id_full_path.substr(pos + 1, zone_id_full_path.length());

      *zone_id = -(std::stoi(zone_id_str));
      std::cout << "Find zone id: " << *zone_id << std::endl;
      return true;
    }
  }
  std::string zone_id_full_path = (*iter_north).path().string();
  std::size_t pos = zone_id_full_path.find_last_of("/");
  std::string zone_id_str =
      zone_id_full_path.substr(pos + 1, zone_id_full_path.length());

  *zone_id = (std::stoi(zone_id_str));
  std::cout << "Find zone id: " << *zone_id << std::endl;
  return true;
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
