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

#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/random.hpp>

#include "absl/strings/str_cat.h"

#include "modules/localization/msf/common/io/velodyne_utility.h"
#include "modules/localization/msf/common/util/extract_ground_plane.h"
#include "modules/localization/msf/common/util/system_utility.h"
#include "modules/localization/msf/local_pyramid_map/ndt_map/ndt_map.h"
#include "modules/localization/msf/local_pyramid_map/ndt_map/ndt_map_pool.h"

using ::apollo::common::EigenAffine3dVec;
using ::apollo::common::EigenVector3dVec;

int main(int argc, char** argv) {
  boost::program_options::options_description boost_desc("Allowed options");
  boost_desc.add_options()("help", "produce help message")(
      "pcd_folders", boost::program_options::value<std::vector<std::string>>(),
      "provide the pcd folders")(
      "pose_files", boost::program_options::value<std::vector<std::string>>(),
      "provide pose files")("map_folder",
                            boost::program_options::value<std::string>(),
                            "provide map folder")(
      "resolution_type", boost::program_options::value<std::string>(),
      "map resolution type: single or multi")(
      "resolution", boost::program_options::value<float>(),
      "provide map resolution")(
      "resolution_z",
      boost::program_options::value<float>()->default_value(0.03125),
      "provide map resolution of z")(
      "zone_id", boost::program_options::value<int>(), "provide zone id")(
      "set_road_cells",
      boost::program_options::value<bool>()->default_value(true),
      "whether enable setting road cells, default true")(
      "pool_size",
      boost::program_options::value<unsigned int>()->default_value(20),
      "set pool size");

  boost::program_options::variables_map boost_args;
  boost::program_options::store(
      boost::program_options::parse_command_line(argc, argv, boost_desc),
      boost_args);
  boost::program_options::notify(boost_args);

  if (boost_args.count("help")) {
    std::cout << boost_desc << std::endl;
    return 0;
  }

  const std::vector<std::string> pcd_folder_paths =
      boost_args["pcd_folders"].as<std::vector<std::string>>();
  const std::vector<std::string> pose_files =
      boost_args["pose_files"].as<std::vector<std::string>>();
  if (pcd_folder_paths.size() != pose_files.size()) {
    std::cerr << "the count of pcd folders is not equal pose files"
              << std::endl;
    return -1;
  }
  const std::string map_base_folder =
      boost_args["map_folder"].as<std::string>();
  const int zone_id = boost_args["zone_id"].as<int>();
  const unsigned int pool_size = boost_args["pool_size"].as<unsigned int>();

  const std::string resolution_type =
      boost_args["resolution_type"].as<std::string>();
  if (strcasecmp(resolution_type.c_str(), "single") != 0 &&
      strcasecmp(resolution_type.c_str(), "multi") != 0) {
    std::cerr << "map resolution type invalid. (single or multi)" << std::endl;
    return -1;
  }

  float single_resolution_map = boost_args["resolution"].as<float>();
  constexpr double epsilon = 1e-8;
  if (std::fabs(single_resolution_map - 0.03125) > epsilon &&
      std::fabs(single_resolution_map - 0.0625) > epsilon &&
      std::fabs(single_resolution_map - 0.125) > epsilon &&
      std::fabs(single_resolution_map - 0.25) > epsilon &&
      std::fabs(single_resolution_map - 0.5) > epsilon &&
      std::fabs(single_resolution_map - 1.0) > epsilon &&
      std::fabs(single_resolution_map - 2.0) > epsilon &&
      std::fabs(single_resolution_map - 4.0) > epsilon &&
      std::fabs(single_resolution_map - 8.0) > epsilon &&
      std::fabs(single_resolution_map - 16.0) > epsilon) {
    std::cerr << "map resolution can only be: 0.03125, "
              << "0.0625, 0.125, 0.25, 0.5, 1.0, 2.0, "
              << "4.0, 8.0 or 16.0." << std::endl;
    return -1;
  }

  float single_resolution_map_z = boost_args["resolution_z"].as<float>();
  std::cout << "single_resolution_map_z: " << single_resolution_map_z
            << std::endl;

  // load all poses
  std::vector<EigenAffine3dVec> pcd_poses(pcd_folder_paths.size());
  std::vector<std::vector<double>> time_stamps(pcd_folder_paths.size());
  std::vector<std::vector<unsigned int>> pcd_indices(pcd_folder_paths.size());
  for (std::size_t i = 0; i < pose_files.size(); ++i) {
    apollo::localization::msf::velodyne::LoadPcdPoses(
        pose_files[i], &pcd_poses[i], &time_stamps[i], &pcd_indices[i]);
  }

  if (!apollo::localization::msf::system::IsExists(map_base_folder)) {
    if (apollo::localization::msf::system::CreateDirectory(map_base_folder)) {
      std::cerr << "Create map directory failed." << std::endl;
    }
  }

  // Output Config file
  apollo::localization::msf::pyramid_map::NdtMapConfig ndt_map_config(
      "map_ndt_v01");
  if (strcasecmp(resolution_type.c_str(), "single") == 0) {
    ndt_map_config.SetSingleResolutions(single_resolution_map);
    ndt_map_config.SetSingleResolutionZ(single_resolution_map_z);
  } else {
    ndt_map_config.SetMultiResolutions();
    ndt_map_config.SetMultiResolutionsZ();
  }
  float distance = 1024 * 0.125;
  unsigned int node_size =
      static_cast<unsigned int>(distance / ndt_map_config.map_resolutions_[0]);
  std::cout << "Node size: " << node_size << std::endl;
  ndt_map_config.map_node_size_x_ = node_size;
  ndt_map_config.map_node_size_y_ = node_size;

  ndt_map_config.map_datasets_.insert(ndt_map_config.map_datasets_.end(),
                                      pcd_folder_paths.begin(),
                                      pcd_folder_paths.end());
  char file_buf[1024];
  snprintf(file_buf, sizeof(file_buf), "%s/config.xml",
           map_base_folder.c_str());
  ndt_map_config.Save(file_buf);

  // Initialize map and pool
  apollo::localization::msf::pyramid_map::NdtMap ndt_map(&ndt_map_config);
  unsigned int thread_size = 4;
  apollo::localization::msf::pyramid_map::NdtMapNodePool ndt_map_node_pool(
      pool_size, thread_size);
  ndt_map_node_pool.Initial(&ndt_map_config);
  ndt_map.InitMapNodeCaches(pool_size / 2, pool_size);
  ndt_map.AttachMapNodePool(&ndt_map_node_pool);
  ndt_map.SetMapFolderPath(map_base_folder);

  // Plane extractor
  apollo::localization::msf::FeatureXYPlane plane_extractor;

  for (unsigned int i = 0; i < pcd_folder_paths.size(); ++i) {
    const EigenAffine3dVec& pcd_poses_i = pcd_poses[i];
    for (unsigned int frame_idx = 0; frame_idx < pcd_poses_i.size();
         ++frame_idx) {
      apollo::localization::msf::velodyne::VelodyneFrame velodyne_frame;
      std::string pcd_file_path = absl::StrCat(
          pcd_folder_paths[i], "/", pcd_indices[i][frame_idx], ".pcd");
      Eigen::Affine3d pcd_pose = pcd_poses_i[frame_idx];
      // Load pcd
      apollo::localization::msf::velodyne::LoadPcds(
          pcd_file_path, pcd_indices[i][frame_idx], pcd_pose, &velodyne_frame,
          false);

      std::cout << "Loaded " << velodyne_frame.pt3ds.size()
                << "3D Points at Trial: " << i
                << " Frame: " << pcd_indices[i][frame_idx] << "." << std::endl;

      // Process the point cloud
      for (size_t k = 0; k < velodyne_frame.pt3ds.size(); ++k) {
        Eigen::Vector3d& pt3d_local = velodyne_frame.pt3ds[k];
        unsigned char intensity = velodyne_frame.intensities[k];
        Eigen::Vector3d pt3d_global = velodyne_frame.pose * pt3d_local;

        for (size_t res = 0; res < ndt_map_config.map_resolutions_.size();
             ++res) {
          apollo::localization::msf::pyramid_map::MapNodeIndex index =
              apollo::localization::msf::pyramid_map::MapNodeIndex::
                  GetMapNodeIndex(ndt_map_config, pt3d_global,
                                  static_cast<unsigned int>(res), zone_id);

          apollo::localization::msf::pyramid_map::NdtMapNode* ndt_map_node =
              static_cast<apollo::localization::msf::pyramid_map::NdtMapNode*>(
                  ndt_map.GetMapNodeSafe(index));
          apollo::localization::msf::pyramid_map::NdtMapMatrix& ndt_map_matrix =
              static_cast<
                  apollo::localization::msf::pyramid_map::NdtMapMatrix&>(
                  ndt_map_node->GetMapCellMatrix());

          Eigen::Vector2d coord2d;
          coord2d[0] = pt3d_global[0];
          coord2d[1] = pt3d_global[1];
          unsigned int x = 0;
          unsigned int y = 0;
          ndt_map_node->GetCoordinate(coord2d, &x, &y);

          // get the centroid
          Eigen::Vector2d left_top_corner = ndt_map_node->GetLeftTopCorner();
          float resolution = ndt_map_node->GetMapResolution();
          Eigen::Vector3f centroid;
          centroid[0] = static_cast<float>(coord2d[0]) -
                        static_cast<float>(left_top_corner[0]) -
                        resolution * static_cast<float>(x);
          centroid[1] = static_cast<float>(coord2d[1]) -
                        static_cast<float>(left_top_corner[1]) -
                        resolution * static_cast<float>(y);
          apollo::localization::msf::pyramid_map::NdtMapCells& ndt_map_cell =
              ndt_map_matrix.GetMapCell(y, x);
          int altitude_index = ndt_map_cell.CalAltitudeIndex(
              ndt_map_config.map_resolutions_z_[res],
              static_cast<float>(pt3d_global[2]));
          centroid[2] = static_cast<float>(pt3d_global[2]) -
                        static_cast<float>(altitude_index) *
                            ndt_map_config.map_resolutions_z_[res];

          ndt_map_cell.AddSample(intensity, static_cast<float>(pt3d_global[2]),
                                 ndt_map_config.map_resolutions_z_[res],
                                 centroid, false);
          ndt_map_node->SetIsChanged(true);
        }
      }
    }
  }

  return 0;
}
