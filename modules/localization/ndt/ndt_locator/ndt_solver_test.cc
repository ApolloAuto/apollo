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

#include "modules/localization/ndt/ndt_locator/ndt_solver.h"

#include "gtest/gtest.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

#include "modules/localization/msf/local_pyramid_map/base_map/base_map_node_index.h"
#include "modules/localization/msf/local_pyramid_map/ndt_map/ndt_map.h"
#include "modules/localization/msf/local_pyramid_map/ndt_map/ndt_map_config.h"
#include "modules/localization/msf/local_pyramid_map/ndt_map/ndt_map_matrix.h"
#include "modules/localization/msf/local_pyramid_map/ndt_map/ndt_map_node.h"
#include "modules/localization/msf/local_pyramid_map/ndt_map/ndt_map_pool.h"

namespace apollo {
namespace localization {
namespace ndt {

typedef apollo::localization::msf::pyramid_map::NdtMap NdtMap;
typedef apollo::localization::msf::pyramid_map::NdtMapConfig NdtMapConfig;
typedef apollo::localization::msf::pyramid_map::NdtMapNode NdtMapNode;
typedef apollo::localization::msf::pyramid_map::NdtMapCells NdtMapCells;
typedef apollo::localization::msf::pyramid_map::NdtMapNodePool NdtMapNodePool;
typedef apollo::localization::msf::pyramid_map::NdtMapMatrix NdtMapMatrix;
typedef apollo::localization::msf::pyramid_map::MapNodeIndex MapNodeIndex;

MapNodeIndex GetMapIndexFromMapFolder(const std::string& map_folder) {
  MapNodeIndex index;
  char buf[100];
  sscanf(map_folder.c_str(), "/%03u/%05s/%02d/%08u/%08u", &index.resolution_id_,
         buf, &index.zone_id_, &index.m_, &index.n_);
  std::string zone = buf;
  if (zone == "south") {
    index.zone_id_ = -index.zone_id_;
  }
  std::cout << index << std::endl;
  return index;
}

bool GetAllMapIndex(const std::string& src_map_folder,
                    std::list<MapNodeIndex>* buf) {
  std::string src_map_path = src_map_folder + "/map";
  buf->clear();
  boost::filesystem::recursive_directory_iterator end_iter;
  boost::filesystem::recursive_directory_iterator iter(src_map_path);
  for (; iter != end_iter; ++iter) {
    if (!boost::filesystem::is_directory(*iter)) {
      if (iter->path().extension() == "") {
        std::string tmp = iter->path().string();
        tmp = tmp.substr(src_map_path.length(), tmp.length());
        buf->push_back(GetMapIndexFromMapFolder(tmp));
      }
    }
  }
  return true;
}

class NdtSolverTestSuite : public ::testing::Test {
 protected:
  NdtSolverTestSuite() {}
  virtual ~NdtSolverTestSuite() {}
  virtual void SetUp() {}
  virtual void TearDown() {}
};

TEST_F(NdtSolverTestSuite, NdtSolver) {
  // Set NDT
  NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> reg;
  reg.SetMaximumIterations(5);
  reg.SetStepSize(0.1);
  reg.SetTransformationEpsilon(0.01);

  // Load input source.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(
      new pcl::PointCloud<pcl::PointXYZ>());
  const std::string input_source_file =
      "/apollo/modules/localization/ndt/test_data/pcds/1.pcd";
  int ret = pcl::io::loadPCDFile(input_source_file, *cloud_source);
  EXPECT_LE(ret, 0);

  // Load input target.
  const std::string map_folder =
      "/apollo/modules/localization/ndt/test_data/ndt_map";
  std::list<MapNodeIndex> buf;
  GetAllMapIndex(map_folder, &buf);
  std::cout << "index size: " << buf.size() << std::endl;

  // Initialize NDT map and pool.
  NdtMapConfig ndt_map_config("map_ndt_v01");
  NdtMap ndt_map(&ndt_map_config);
  ndt_map.SetMapFolderPath(map_folder);
  NdtMapNodePool ndt_map_node_pool(20, 4);
  ndt_map_node_pool.Initial(&ndt_map_config);
  ndt_map.InitMapNodeCaches(10, 4);
  ndt_map.AttachMapNodePool(&ndt_map_node_pool);

  // Get the map pointcloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cell_pointcloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  std::vector<Leaf> cell_map;
  Eigen::Vector2d map_left_top_corner(Eigen::Vector2d::Zero());

  int index = 0;
  for (auto itr = buf.begin(); itr != buf.end(); ++itr, ++index) {
    NdtMapNode* ndt_map_node =
        static_cast<NdtMapNode*>(ndt_map.GetMapNodeSafe(*itr));
    if (ndt_map_node == nullptr) {
      AERROR << "index: " << index << " is a NULL pointer!" << std::endl;
      continue;
    }
    NdtMapMatrix& ndt_map_matrix =
        static_cast<NdtMapMatrix&>(ndt_map_node->GetMapCellMatrix());
    const Eigen::Vector2d& left_top_corner = ndt_map_node->GetLeftTopCorner();
    double resolution = ndt_map_node->GetMapResolution();
    double resolution_z = ndt_map_node->GetMapResolutionZ();

    if (index == 0) {
      map_left_top_corner = left_top_corner;
    }
    if (left_top_corner(0) < map_left_top_corner(0) &&
        left_top_corner(1) < map_left_top_corner(1)) {
      map_left_top_corner = left_top_corner;
    }

    int rows = ndt_map_config.map_node_size_y_;
    int cols = ndt_map_config.map_node_size_x_;
    for (int row = 0; row < rows; ++row) {
      for (int col = 0; col < cols; ++col) {
        const NdtMapCells& cell_ndt = ndt_map_matrix.GetMapCell(row, col);
        for (auto it = cell_ndt.cells_.begin(); it != cell_ndt.cells_.end();
             ++it) {
          unsigned int cell_count = it->second.count_;

          if (cell_count >= 6 && it->second.is_icov_available_) {
            Leaf leaf;
            leaf.nr_points_ = static_cast<int>(cell_count);

            Eigen::Vector3d point(Eigen::Vector3d::Zero());
            point(0) = left_top_corner(0) +
                       (static_cast<double>(col)) * resolution +
                       static_cast<double>(it->second.centroid_[0]);
            point(1) = left_top_corner(1) +
                       (static_cast<double>(row)) * resolution +
                       static_cast<double>(it->second.centroid_[1]);
            point(2) = resolution_z * static_cast<double>(it->first) +
                       static_cast<double>(it->second.centroid_[2]);
            leaf.mean_ = point;
            if (it->second.is_icov_available_ == 1) {
              leaf.icov_ = it->second.centroid_icov_.cast<double>();
            } else {
              leaf.nr_points_ = -1;
            }
            cell_map.push_back(leaf);
            cell_pointcloud->push_back(pcl::PointXYZ(
                static_cast<float>(point(0)), static_cast<float>(point(1)),
                static_cast<float>(point(2))));
          }
        }
      }
    }
  }

  // Set left top corner.
  Eigen::Vector3d target_left_top_corner(Eigen::Vector3d::Zero());
  target_left_top_corner.block<2, 1>(0, 0) = map_left_top_corner;
  reg.SetLeftTopCorner(target_left_top_corner);

  // Set input target.
  reg.SetResolution(ndt_map_config.map_resolutions_[0]);
  reg.SetInputTarget(cell_map, cell_pointcloud);

  // Set input source.
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_source);
  sor.setLeafSize(1.0, 1.0, 1.0);
  sor.filter(*cloud_source);
  reg.SetInputSource(cloud_source);

  // Align
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Quaterniond quat =
      Eigen::Quaterniond(0.857989, 0.009698, -0.008629, -0.513505);
  Eigen::Vector3d translation =
      Eigen::Vector3d(588348.947978, 4141240.223859, -30.094324);
  Eigen::Vector3d error = Eigen::Vector3d(0.5, -0.5, 0.3);
  Eigen::Matrix4d transform(Eigen::Matrix4d::Identity());
  transform.block<3, 3>(0, 0) = quat.toRotationMatrix();
  transform.block<3, 1>(0, 3) = translation + error;
  reg.Align(output_cloud, transform.cast<float>());
  EXPECT_EQ(output_cloud->points.size(), cloud_source->points.size());

  // Result
  double fitness_score = reg.GetFitnessScore();
  bool has_converged = reg.HasConverged();
  int iteration = reg.GetFinalNumIteration();
  Eigen::Matrix4f ndt_pose = reg.GetFinalTransformation();
  AINFO << ndt_pose(0, 3) << ", " << ndt_pose(1, 3) << ", " << ndt_pose(2, 3)
        << std::endl;
  ASSERT_LE(fitness_score, 2.0);
  ASSERT_TRUE(has_converged);
  ASSERT_LE(iteration, 7);
}

}  // namespace ndt
}  // namespace localization
}  // namespace apollo
