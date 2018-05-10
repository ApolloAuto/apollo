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

#include <cstddef>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "pcl/io/pcd_io.h"

#include "modules/common/util/file.h"
#include "modules/perception/common/pcl_types.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/obstacle/common/pose_util.h"
#include "modules/perception/obstacle/lidar/visualizer/opengl_visualizer/frame_content.h"
#include "modules/perception/obstacle/lidar/visualizer/opengl_visualizer/opengl_visualizer.h"
#include "modules/perception/obstacle/onboard/lidar_process.h"

DECLARE_string(flagfile);
DECLARE_bool(enable_visualization);
DECLARE_string(config_manager_path);
DEFINE_string(pcd_path, "./pcd/", "pcd path");
DEFINE_string(pose_path, "./pose/", "pose path");
DEFINE_string(output_path, "./output/", "output path");
DEFINE_bool(save_obstacles, true, "save obstacles to file");

namespace apollo {
namespace perception {

DEFINE_int32(start_frame, 1, "start frame");

class OfflineLidarPerceptionTool {
 public:
  bool Init(bool use_visualization = false) {
    if (!ConfigManager::instance()->Init()) {
      AERROR << "failed to Init ConfigManager";
      return false;
    }

    lidar_process_.reset(new LidarProcess());
    if (!lidar_process_->Init()) {
      AERROR << "failed to Init lidar_process.";
      return false;
    }

    if (use_visualization) {
      visualizer_.reset(new OpenglVisualizer());
      if (!visualizer_->Init()) {
        AERROR << "Init visialuzer failed" << std::endl;
      }
    }
    return true;
  }

  void Run(const std::string& pcd_path, const std::string& pose_path,
           const std::string& output_path) {
    std::string pcd_folder = pcd_path;
    std::string pose_folder = pose_path;
    std::vector<std::string> pcd_file_names;
    std::vector<std::string> pose_file_names;
    AINFO << "starting to run";
    common::util::GetFileNamesInFolderById(pose_folder, ".pose",
                                           &pose_file_names);
    common::util::GetFileNamesInFolderById(pcd_folder, ".pcd", &pcd_file_names);
    AINFO << " pose size " << pose_file_names.size();
    AINFO << " pcd size " << pcd_file_names.size();
    if (pose_file_names.size() != pcd_file_names.size()) {
      AERROR << "pcd file number does not match pose file number";
      return;
    }
    double time_stamp = 0.0;
    int start_frame = FLAGS_start_frame;
    AINFO << "starting frame is " << start_frame;
    sleep(1);
    for (size_t i = 0; i < pcd_file_names.size(); i++) {
      AINFO << "***************** Frame " << i << " ******************";
      std::ostringstream oss;
      pcl_util::PointCloudPtr cloud(new pcl_util::PointCloud);
      AINFO << "load pcd file from file path" << pcd_folder + pcd_file_names[i];
      pcl::io::loadPCDFile<pcl_util::Point>(pcd_folder + pcd_file_names[i],
                                            *cloud);

      // read pose
      Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
      int frame_id = -1;
      if (!ReadPoseFile(pose_folder + pose_file_names[i], &pose, &frame_id,
                        &time_stamp)) {
        AERROR << "Failed to read pose file" << pose_file_names[i];
        return;
      }

      auto velodyne_trans = std::make_shared<Eigen::Matrix4d>(pose);
      lidar_process_->Process(time_stamp, cloud, velodyne_trans);

      std::vector<std::shared_ptr<Object>> result_objects =
          lidar_process_->GetObjects();
      const pcl_util::PointIndicesPtr roi_indices =
          lidar_process_->GetROIIndices();

      pcl_util::PointCloudPtr roi_cloud(new pcl_util::PointCloud);
      pcl::copyPointCloud(*cloud, *roi_indices, *roi_cloud);

      if (visualizer_) {
        pcl_util::PointDCloudPtr transformed_cloud(new pcl_util::PointDCloud);
        TransformPointCloud(cloud, pose, transformed_cloud);
        AERROR << "transformed cloud size is " << transformed_cloud->size();

        pcl_util::PointIndices roi_indices_1;
        FrameContent content;
        content.SetLidarPose(pose);
        content.SetLidarCloud(cloud);
        content.SetLidarRoiCloud(roi_cloud);
        content.SetTrackedObjects(result_objects);
        visualizer_->UpdateCameraSystem(&content);
        visualizer_->Render(content);
      }
      AINFO << "finish pc";

      if (FLAGS_save_obstacles) {
        oss << std::setfill('0') << std::setw(6) << i;
        std::string filename = FLAGS_output_path + oss.str() + ".txt";
        SaveTrackingInformation(&result_objects, pose, frame_id, cloud,
                                filename);
      }
    }
  }

  void SaveTrackingInformation(std::vector<std::shared_ptr<Object>>* objects,
                               const Eigen::Matrix4d& pose_v2w,
                               const int& frame_id,
                               const pcl_util::PointCloudPtr& cloud,
                               const std::string& filename) {
    std::ofstream fout(filename.c_str(), std::ios::out);
    if (!fout) {
      AERROR << filename << " is not exist!";
      return;
    }
    // write frame id & number of objects at the beignning
    fout << frame_id << " " << objects->size() << std::endl;

    typename pcl::PointCloud<pcl_util::Point>::Ptr trans_cloud(
        new pcl::PointCloud<pcl_util::Point>());
    Eigen::Matrix4d pose_velo2tw = pose_v2w;
    pcl::copyPointCloud(*cloud, *trans_cloud);
    TransformPointCloud<pcl_util::Point>(pose_v2w, trans_cloud);
    AINFO << "point size of transforming cloud is " << trans_cloud->size();
    pcl::KdTreeFLANN<pcl_util::Point> pcl_kdtree;
    pcl_kdtree.setInputCloud(trans_cloud);
    std::vector<int> k_indices;
    std::vector<float> k_sqrt_dist;
    Eigen::Matrix4d pose_tw2velo = pose_velo2tw.inverse();

    for (const auto& obj : *objects) {
      Eigen::Vector3f coord_dir(0.0, 1.0, 0.0);
      Eigen::Vector4d dir_velo =
          pose_tw2velo * Eigen::Vector4d(obj->direction[0], obj->direction[1],
                                         obj->direction[2], 0);
      Eigen::Vector4d ct_velo =
          pose_tw2velo *
          Eigen::Vector4d(obj->center[0], obj->center[1], obj->center[2], 1);
      Eigen::Vector3f dir_velo3(dir_velo[0], dir_velo[1], dir_velo[2]);
      double theta = VectorTheta2dXy(coord_dir, dir_velo3);
      std::string type = "unknown";
      if (obj->type == ObjectType::PEDESTRIAN) {
        type = "pedestrain";
      } else if (obj->type == ObjectType::VEHICLE) {
        type = "smallMot";
      } else if (obj->type == ObjectType::BICYCLE) {
        type = "nonMot";
      }
      // write tracking details
      fout << obj->id << " " << obj->track_id << " " << type << " "
           << std::setprecision(10) << ct_velo[0] << " " << ct_velo[1] << " "
           << ct_velo[2] << " " << obj->length << " " << obj->width << " "
           << obj->height << " " << theta << " " << 0 << " " << 0 << " "
           << obj->velocity[0] << " " << obj->velocity[1] << " "
           << obj->velocity[2] << " " << obj->cloud->size() << " ";

      for (const pcl_util::Point& pt : *obj->cloud) {
        pcl_util::Point query_pt;
        query_pt.x = pt.x;
        query_pt.y = pt.y;
        query_pt.z = pt.z;
        k_indices.resize(1);
        k_sqrt_dist.resize(1);
        pcl_kdtree.nearestKSearch(query_pt, 1, k_indices, k_sqrt_dist);
        fout << k_indices[0] << " ";
      }
      fout << std::endl;
    }
    fout.close();
  }

 protected:
  std::unique_ptr<LidarProcess> lidar_process_;
  std::unique_ptr<OpenglVisualizer> visualizer_;
};

}  // namespace perception
}  // namespace apollo

int main(int argc, char* argv[]) {
  FLAGS_flagfile =
      "./modules/perception/tool/offline_visualizer_tool/conf/"
      "offline_lidar_perception_test.flag";
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  apollo::perception::OfflineLidarPerceptionTool tool;
  tool.Init(FLAGS_enable_visualization);
  tool.Run(FLAGS_pcd_path, FLAGS_pose_path, FLAGS_output_path);
  return 0;
}
