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

/**
 * @file velodyne_utility.h
 * @brief The utilities of velodyne.
 */

#ifndef MODULES_LOCALIZATION_MSF_COMMON_VELODYNE_UTILITY_H_
#define MODULES_LOCALIZATION_MSF_COMMON_VELODYNE_UTILITY_H_

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace apollo {
namespace localization {
namespace msf {
namespace velodyne {

struct VelodyneFrame {
  /**@brief The frame index. */
  unsigned int frame_index;
  /**@brief The time stamp. */
  double timestamp;
  /**@brief The 3D point cloud in this frame. */
  std::vector<Eigen::Vector3d> pt3ds;
  /**@brief The laser reflection values in this frames. */
  std::vector<unsigned char> intensities;
  /**@brief The laser IDs. */
  std::vector<unsigned char> laser_ids;
  /**@brief The pose of the frame. */
  Eigen::Affine3d pose;
};

void load_pcds(std::string file_path, 
    unsigned int frame_index,
    const Eigen::Affine3d& pose,
    std::vector<Eigen::Vector3d>& pt3ds, 
    std::vector<unsigned char>& intensities,
    bool is_global);

/**@brief Load the PCD poses with their timestamps. */
void load_pcd_poses(std::string file_path,
    std::vector<Eigen::Affine3d>& poses,
    std::vector<double>& timestamps);

// /**@brief Save the PCD poses with their timestamps. */
// void save_pcd_poses(std::string file_path,
//    const std::vector<Eigen::Affine3d>& poses,
//    const std::vector<double>& timestamps);

/**@brief Load the velodyne extrinsic from a YAML file. */
bool load_extrinsic(std::string file_path, Eigen::Affine3d& extrinsic);

} // namespace velodyne
} // namespace msf
} // namespace localization
} // namespace apollo

#endif // MODULES_LOCALIZATION_MSF_COMMON_VELODYNE_UTILITY_H_

