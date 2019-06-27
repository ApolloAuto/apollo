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

#include <fstream>

#include "cyber/common/log.h"
#include "modules/localization/msf/common/io/velodyne_utility.h"
#include "modules/localization/msf/local_tool/map_creation/poses_interpolation/poses_interpolation.h"

namespace apollo {
namespace localization {
namespace msf {
PosesInterpolation::PosesInterpolation() {}

bool PosesInterpolation::Init(const std::string &input_poses_path,
                              const std::string &ref_timestamps_path,
                              const std::string &out_poses_path,
                              const std::string &extrinsic_path) {
  this->input_poses_path_ = input_poses_path;
  this->ref_timestamps_path_ = ref_timestamps_path;
  this->out_poses_path_ = out_poses_path;
  this->extrinsic_path_ = extrinsic_path;

  bool success = velodyne::LoadExtrinsic(extrinsic_path_, &velodyne_extrinsic_);
  if (!success) {
    AERROR << "Load lidar extrinsic failed.";
    return false;
  }

  return true;
}

void PosesInterpolation::DoInterpolation() {
  // Load input poses
  std::vector<Eigen::Vector3d> input_stds;
  velodyne::LoadPosesAndStds(input_poses_path_, &input_poses_, &input_stds,
                             &input_poses_timestamps_);

  // Load pcd timestamp
  LoadPCDTimestamp();

  // Interpolation
  PoseInterpolationByTime(input_poses_, input_poses_timestamps_,
                          ref_timestamps_, ref_ids_, &out_indexes_,
                          &out_timestamps_, &out_poses_);

  // Write pcd poses
  WritePCDPoses();
}

void PosesInterpolation::LoadPCDTimestamp() {
  FILE *file = fopen(ref_timestamps_path_.c_str(), "r");
  if (file) {
    unsigned int index;
    double timestamp;
    constexpr int kSize = 2;
    while (fscanf(file, "%u %lf\n", &index, &timestamp) == kSize) {
      ref_timestamps_.push_back(timestamp);
      ref_ids_.push_back(index);
    }
    fclose(file);
  } else {
    AINFO << "Can't open file to read: " << ref_timestamps_path_;
  }
}

void PosesInterpolation::WritePCDPoses() {
  std::ofstream fout;
  fout.open(out_poses_path_.c_str(), std::ofstream::out);

  if (fout.is_open()) {
    fout.setf(std::ios::fixed, std::ios::floatfield);
    fout.precision(6);
    for (size_t i = 0; i < out_poses_.size(); i++) {
      double timestamp = out_timestamps_[i];

      Eigen::Affine3d pose_tem = out_poses_[i] * velodyne_extrinsic_;
      Eigen::Quaterniond quatd(pose_tem.linear());
      Eigen::Translation3d transd(pose_tem.translation());
      double qx = quatd.x();
      double qy = quatd.y();
      double qz = quatd.z();
      double qr = quatd.w();

      fout << out_indexes_[i] << " " << timestamp << " " << transd.x() << " "
           << transd.y() << " " << transd.z() << " " << qx << " " << qy << " "
           << qz << " " << qr << "\n";
    }

    fout.close();
  } else {
    AERROR << "Can't open file to write: " << out_poses_path_ << std::endl;
  }
}  // namespace msf

void PosesInterpolation::PoseInterpolationByTime(
    const std::vector<Eigen::Affine3d> &in_poses,
    const std::vector<double> &in_timestamps,
    const std::vector<double> &ref_timestamps,
    const std::vector<unsigned int> &ref_indexes,
    std::vector<unsigned int> *out_indexes, std::vector<double> *out_timestamps,
    std::vector<Eigen::Affine3d> *out_poses) {
  out_indexes->clear();
  out_timestamps->clear();
  out_poses->clear();

  unsigned int index = 0;
  for (size_t i = 0; i < ref_timestamps.size(); i++) {
    double ref_timestamp = ref_timestamps[i];
    unsigned int ref_index = ref_indexes[i];

    while (index < in_timestamps.size() &&
           in_timestamps.at(index) < ref_timestamp) {
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

        out_poses->push_back(re_transd * res_quatd);
        out_indexes->push_back(ref_index);
        out_timestamps->push_back(ref_timestamp);
      }
    } else {
      AWARN << "[WARN] No more poses. Exit now.";
      break;
    }
    ADEBUG << "Frame_id: " << i;
  }
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
