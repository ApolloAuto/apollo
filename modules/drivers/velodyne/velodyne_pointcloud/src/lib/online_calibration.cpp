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

#include "velodyne_pointcloud/online_calibration.h"

namespace apollo {
namespace drivers {
namespace velodyne {

int OnlineCalibration::decode(
    const velodyne_msgs::VelodyneScanUnified::ConstPtr& scan_msgs) {
  if (inited_) {
    return 0;
  }
  for (auto& packet : scan_msgs->packets) {
    status_types_.emplace_back(packet.data[1204]);
    status_values_.emplace_back(packet.data[1205]);
  }
  // read calibration when get 2s packet
  if (status_types_.size() < 5789 * 2) {
    ROS_INFO("Wait for more scan msgs");
    return -1;
  }
  get_unit_index();
  int unit_size = unit_indexs_.size();
  if (unit_size < 2) {
    // can not find two unit# index, may be lost packet
    ROS_ERROR_STREAM("unit count less than 2, maybe lost packets");
    return -1;
  }

  if (unit_indexs_[unit_size - 1] - unit_indexs_[unit_size - 2] !=
      65 * 64) {  // 64 lasers
    // lost packet
    ROS_ERROR_STREAM("two unit distance is wrong");
    return -1;
  }

  int start_index = unit_indexs_[unit_size - 2];
  for (int i = 0; i < 64; ++i) {
    LaserCorrection laser_correction;

    int index_16 = start_index + i * 64 + 16;
    int index_32 = start_index + i * 64 + 32;
    int index_48 = start_index + i * 64 + 48;

    laser_correction.laser_ring = status_values_[index_16];

    laser_correction.vert_correction =
        *(int16_t*)(&status_values_[index_16 + 1]) / 100.0 * DEGRESS_TO_RADIANS;
    laser_correction.rot_correction =
        *(int16_t*)(&status_values_[index_16 + 3]) / 100.0 * DEGRESS_TO_RADIANS;
    laser_correction.dist_correction =
        *(int16_t*)(&status_values_[index_16 + 5]) / 10.0 / 100.0;  // to meter
    laser_correction.dist_correction_x =
        *(int16_t*)(&status_values_[index_32]) / 10.0 / 100.0;  // to meter
    laser_correction.dist_correction_y =
        *(int16_t*)(&status_values_[index_32 + 2]) / 10.0 / 100.0;  // to meter
    laser_correction.vert_offset_correction =
        *(int16_t*)(&status_values_[index_32 + 4]) / 10.0 / 100.0;  // to meter
    laser_correction.horiz_offset_correction =
        (int16_t)((int16_t)status_values_[index_48] << 8 |
                  status_values_[index_32 + 6]) /
        10.0 / 100.0;  // to meter
    laser_correction.focal_distance =
        *(int16_t*)(&status_values_[index_48 + 1]) / 10.0 / 100.0;  // to meter
    laser_correction.focal_slope =
        *(int16_t*)(&status_values_[index_48 + 3]) / 10.0;  // to meter
    laser_correction.max_intensity = status_values_[index_48 + 6];
    laser_correction.min_intensity = status_values_[index_48 + 5];

    laser_correction.cos_rot_correction = cosf(laser_correction.rot_correction);
    laser_correction.sin_rot_correction = sinf(laser_correction.rot_correction);
    laser_correction.cos_vert_correction =
        cosf(laser_correction.vert_correction);
    laser_correction.sin_vert_correction =
        sinf(laser_correction.vert_correction);
    laser_correction.focal_offset =
        256 * pow(1 - laser_correction.focal_distance / 13100, 2);

    calibration_.laser_corrections_[laser_correction.laser_ring] =
        laser_correction;
  }
  calibration_.num_lasers_ = 64;
  calibration_.initialized_ = true;
  inited_ = true;
  return 0;
}

void OnlineCalibration::get_unit_index() {
  int size = status_values_.size();
  // simple check only for value, maybe need more check fro status tyep
  int start_index = 0;
  if (unit_indexs_.size() > 0) {
    start_index = unit_indexs_.back() + 5;
  }
  for (int start_index = 0; start_index < size - 5; ++start_index) {
    if (status_values_[start_index] == 85            // "U"
        && status_values_[start_index + 1] == 78     // "N"
        && status_values_[start_index + 2] == 73     // "I"
        && status_values_[start_index + 3] == 84     // "T"
        && status_values_[start_index + 4] == 35) {  // "#"
      unit_indexs_.emplace_back(start_index);
    }
  }
}

void OnlineCalibration::dump(const std::string& file_path) {
  if (!inited_) {
    ROS_ERROR("Please decode calibraion info first");
    return;
  }
  std::ofstream ofs(file_path.c_str(), std::ios::out);
  ofs << "lasers:" << std::endl;
  for (auto& correction : calibration_.laser_corrections_) {
    ofs << "- {";
    ofs << "dist_correction: " << correction.second.dist_correction << ", ";
    ofs << "dist_correction_x: " << correction.second.dist_correction_x << ", ";
    ofs << "dist_correction_y: " << correction.second.dist_correction_y << ", ";
    ofs << "focal_distance: " << correction.second.focal_distance << ", ";
    ofs << "focal_slope: " << correction.second.focal_slope << ", ";
    ofs << "horiz_offset_correction: "
        << correction.second.horiz_offset_correction << ", ";
    ofs << "laser_id: " << correction.second.laser_ring << ", ";
    ofs << "max_intensity: " << correction.second.max_intensity << ", ";
    ofs << "min_intensity: " << correction.second.min_intensity << ", ";
    ofs << "rot_correction: " << correction.second.rot_correction << ", ";
    ofs << "vert_correction: " << correction.second.vert_correction << ", ";
    ofs << "vert_offset_correction: "
        << correction.second.vert_offset_correction;
    ofs << "}" << std::endl;
  }
  ofs << "num_lasers: " << calibration_.num_lasers_ << std::endl;
  ofs.close();
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo