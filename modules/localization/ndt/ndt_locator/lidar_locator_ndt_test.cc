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

#include "modules/localization/ndt/ndt_locator/lidar_locator_ndt.h"

#include <iostream>
#include <sstream>

#include <boost/filesystem.hpp>
#include "gtest/gtest.h"
#include "pcl/io/pcd_io.h"

#include "cyber/common/log.h"
#include "modules/localization/msf/common/io/pcl_point_types.h"
#include "modules/localization/msf/common/io/velodyne_utility.h"

namespace apollo {
namespace localization {
namespace ndt {

TEST(LidarLocatorNdtTestSuite, LidarLocatorNdt) {
  const std::string map_folder =
      "/apollo/modules/localization/ndt/test_data/ndt_map";
  const std::string poses_file =
      "/apollo/modules/localization/ndt/test_data/pcds/poses.txt";
  LidarLocatorNdt locator;

  // Locator settings.
  locator.SetMapFolderPath(map_folder);
  Eigen::Affine3d velodyne_extrinsic(Eigen::Matrix4d::Identity());
  locator.SetVelodyneExtrinsic(velodyne_extrinsic);
  locator.SetOnlineCloudResolution(1.0);
  locator.SetLidarHeight(1.7);

  // Load poses.
  std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> poses;
  std::vector<double> timestamps;
  msf::velodyne::LoadPcdPoses(poses_file, &poses, &timestamps);
  Eigen::Affine3d location = Eigen::Affine3d::Identity();

  unsigned int frame_idx = 0;
  for (; frame_idx < poses.size(); ++frame_idx) {
    // Config.
    const unsigned int resolution_id = 0;
    const int zone_id = 10;
    if (!locator.IsInitialized()) {
      locator.Init(poses[frame_idx], resolution_id, zone_id);
      continue;
    }

    ::apollo::common::EigenVector3dVec pt3ds;
    std::vector<unsigned char> intensities;
    std::stringstream ss;
    ss << frame_idx + 1;
    std::string pcd_file_path =
        "/apollo/modules/localization/ndt/test_data/pcds/" + ss.str() + ".pcd";
    msf::velodyne::LoadPcds(pcd_file_path, frame_idx, poses[frame_idx], &pt3ds,
                            &intensities);
    LidarFrame lidar_frame;
    lidar_frame.measurement_time = timestamps[frame_idx];
    lidar_frame.pt_xs.reserve(pt3ds.size());
    lidar_frame.pt_ys.reserve(pt3ds.size());
    lidar_frame.pt_zs.reserve(pt3ds.size());
    lidar_frame.intensities.reserve(pt3ds.size());
    for (unsigned int i = 0; i < pt3ds.size(); ++i) {
      lidar_frame.pt_xs.push_back(static_cast<float>(pt3ds[i][0]));
      lidar_frame.pt_ys.push_back(static_cast<float>(pt3ds[i][1]));
      lidar_frame.pt_zs.push_back(static_cast<float>(pt3ds[i][2]));
      lidar_frame.intensities.push_back(intensities[i]);
    }
    int ret = locator.Update(0, poses[frame_idx], lidar_frame);
    EXPECT_LE(std::fabs(ret - 0), 1e-6);

    location = locator.GetPose();
  }
  EXPECT_LE(std::fabs(location.translation()[0] - 588349.337377345), 0.1);
  EXPECT_LE(std::fabs(location.translation()[1] - 4141239.53859664), 0.1);
  EXPECT_LE(std::fabs(location.translation()[2] + 30.0964486966756), 0.1);
  EXPECT_EQ(frame_idx, 2);
}

}  // namespace ndt
}  // namespace localization
}  // namespace apollo
