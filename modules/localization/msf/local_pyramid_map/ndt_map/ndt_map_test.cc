/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/localization/msf/local_pyramid_map/ndt_map/ndt_map.h"

#include <string>
#include <vector>

#include "absl/strings/str_cat.h"
#include "gtest/gtest.h"

#include "cyber/common/file.h"
#include "modules/localization/msf/common/io/pcl_point_types.h"
#include "modules/localization/msf/common/io/velodyne_utility.h"
#include "modules/localization/msf/local_pyramid_map/ndt_map/ndt_map_config.h"
#include "modules/localization/msf/local_pyramid_map/ndt_map/ndt_map_matrix.h"
#include "modules/localization/msf/local_pyramid_map/ndt_map/ndt_map_pool.h"

namespace apollo {
namespace localization {
namespace msf {
namespace pyramid_map {

TEST(MapNdtTestSuite, matrix) {
  const std::string pcd_folder =
      "/apollo/modules/localization/msf/local_map/test_data/ndt_map/pcds";
  EXPECT_TRUE(apollo::cyber::common::EnsureDirectory(pcd_folder));
  const std::string pose_files = pcd_folder + "/poses.txt";

  const std::string map_base_folder =
      "/apollo/modules/localization/msf/local_map/test_data/ndt_map/map_data";
  EXPECT_TRUE(apollo::cyber::common::EnsureDirectory(map_base_folder));

  ::apollo::common::EigenAffine3dVec pcd_poses;
  std::vector<double> time_stamps;
  std::vector<unsigned int> pcd_indices;
  velodyne::LoadPcdPoses(pose_files, &pcd_poses, &time_stamps, &pcd_indices);

  // Create config file
  NdtMapConfig map_config("map_ndt_v01");
  map_config.SetSingleResolutions(1.0);
  map_config.SetSingleResolutionZ(1.0);
  EXPECT_DOUBLE_EQ(map_config.map_resolutions_z_[0], 1.0);
  float distance = 1024 * 0.125;
  unsigned int node_size =
      static_cast<unsigned int>(distance / map_config.map_resolutions_[0]);
  map_config.map_node_size_x_ = node_size;
  map_config.map_node_size_y_ = node_size;
  map_config.map_datasets_.push_back(map_base_folder);
  char file_buf[1024];
  snprintf(file_buf, sizeof(file_buf), "%s/config.xml",
           map_base_folder.c_str());
  EXPECT_TRUE(map_config.Save(file_buf));
  const int zone_id = 10;

  // Initialize map and pool
  NdtMap ndt_map(&map_config);
  unsigned int thread_size = 4;
  unsigned int pool_size = 20;
  NdtMapNodePool ndt_node_pool(pool_size, thread_size);
  ndt_node_pool.Initial(&map_config);
  ndt_map.InitMapNodeCaches(pool_size / 2, pool_size);
  ndt_map.AttachMapNodePool(&ndt_node_pool);
  ndt_map.SetMapFolderPath(map_base_folder);

  for (unsigned int frame_idx = 0; frame_idx < pcd_poses.size(); ++frame_idx) {
    const std::string pcd_file_path =
        absl::StrCat(pcd_folder, "/", pcd_indices[frame_idx], ".pcd");
    velodyne::VelodyneFrame velodyne_frame;
    velodyne::LoadPcds(pcd_file_path, pcd_indices[frame_idx],
                       pcd_poses[frame_idx], &velodyne_frame, false);
    Eigen::Affine3d velo_poses = velodyne_frame.pose;

    for (size_t k = 0; k < velodyne_frame.pt3ds.size(); ++k) {
      Eigen::Vector3d& pt3d_local = velodyne_frame.pt3ds[k];
      unsigned char intensity = velodyne_frame.intensities[k];
      Eigen::Vector3d pt3d_global = velo_poses * pt3d_local;

      for (size_t res = 0; res < map_config.map_resolutions_.size(); ++res) {
        MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(
            map_config, pt3d_global, static_cast<unsigned int>(res), zone_id);
        NdtMapNode* ndt_map_node =
            static_cast<NdtMapNode*>(ndt_map.GetMapNodeSafe(index));
        NdtMapMatrix& ndt_map_matrix =
            static_cast<NdtMapMatrix&>(ndt_map_node->GetMapCellMatrix());

        const Eigen::Vector2d coord2d(pt3d_global[0], pt3d_global[1]);
        unsigned int x = 0;
        unsigned int y = 0;
        ndt_map_node->GetCoordinate(coord2d, &x, &y);

        // get the centroid
        Eigen::Vector2d left_top_corner = ndt_map_node->GetLeftTopCorner();
        const float resolution = ndt_map_node->GetMapResolution();
        NdtMapCells& ndt_map_cells = ndt_map_matrix.GetMapCell(y, x);
        int altitude_index =
            ndt_map_cells.CalAltitudeIndex(map_config.map_resolutions_z_[res],
                                           static_cast<float>(pt3d_global[2]));
        const Eigen::Vector3f centroid(
            static_cast<float>(coord2d[0]) -
                static_cast<float>(left_top_corner[0]) -
                resolution * static_cast<float>(x),
            static_cast<float>(coord2d[1]) -
                static_cast<float>(left_top_corner[1]) -
                resolution * static_cast<float>(y),
            static_cast<float>(pt3d_global[2]) -
                static_cast<float>(altitude_index) *
                    map_config.map_resolutions_z_[res]);

        // Add sample 3d points
        ndt_map_cells.AddSample(intensity, static_cast<float>(pt3d_global[2]),
                                map_config.map_resolutions_z_[res], centroid,
                                false);
        ndt_map_node->SetIsChanged(true);
      }
    }
  }
}

}  // namespace pyramid_map
}  // namespace msf
}  // namespace localization
}  // namespace apollo
