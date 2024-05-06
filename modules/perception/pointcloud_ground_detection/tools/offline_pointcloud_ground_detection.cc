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

#include <memory>
#include <string>

#include "Eigen/Dense"
#include "gflags/gflags.h"

#include "cyber/common/file.h"
#include "modules/perception/common/algorithm/io/io_util.h"
#include "modules/perception/common/base/object_pool_types.h"
#include "modules/perception/common/lidar/common/lidar_frame_pool.h"
#include "modules/perception/common/lidar/common/lidar_point_label.h"
#include "modules/perception/common/lidar/common/pcl_util.h"
#include "modules/perception/pointcloud_ground_detection/interface/base_ground_detector.h"

DEFINE_string(pcd_path, "./pcd/", "pcd path");
DEFINE_string(output_path, "./output/", "output path");
DEFINE_string(ground_detector_name, "SpatioTemporalGroundDetector",
              "ground detector name");
DEFINE_string(config_path, "perception/pointcloud_ground_detection/data",
              "config path");
DEFINE_string(config_file, "spatio_temporal_ground_detector.pb.txt",
              "config file");

namespace apollo {
namespace perception {
namespace lidar {

class OfflinePointcloudGroundDetection {
 public:
  OfflinePointcloudGroundDetection() = default;

  ~OfflinePointcloudGroundDetection() = default;

  bool Init() {
    BaseGroundDetector* ground_detector =
        BaseGroundDetectorRegisterer::GetInstanceByName(
            FLAGS_ground_detector_name);
    CHECK_NOTNULL(ground_detector);
    ground_detector_.reset(ground_detector);

    GroundDetectorInitOptions ground_detector_init_options;
    ground_detector_init_options.config_path = FLAGS_config_path;
    ground_detector_init_options.config_file = FLAGS_config_file;
    ACHECK(ground_detector_->Init(ground_detector_init_options))
        << "Failed to init ground detection.";

    return true;
  }

  bool TransformCloud(const base::PointFCloudPtr& local_cloud,
                      const Eigen::Affine3d& pose,
                      base::PointDCloudPtr world_cloud) const {
    if (local_cloud == nullptr) {
      return false;
    }
    world_cloud->clear();
    world_cloud->reserve(local_cloud->size());
    for (size_t i = 0; i < local_cloud->size(); ++i) {
      auto& pt = local_cloud->at(i);
      Eigen::Vector3d trans_point(pt.x, pt.y, pt.z);
      trans_point = pose * trans_point;
      base::PointD world_point;
      world_point.x = trans_point(0);
      world_point.y = trans_point(1);
      world_point.z = trans_point(2);
      world_point.intensity = pt.intensity;
      world_cloud->push_back(world_point, local_cloud->points_timestamp(i),
                             std::numeric_limits<float>::max(),
                             local_cloud->points_beam_id()[i], 0);
    }
    return true;
  }

  bool Run() {
    std::string pcd_folder = FLAGS_pcd_path;
    std::vector<std::string> pcd_file_names;
    if (!algorithm::GetFileList(pcd_folder, ".pcd", &pcd_file_names)) {
      AERROR << "pcd_folder: " << pcd_folder << " get file list error.";
      return false;
    }

    std::sort(pcd_file_names.begin(), pcd_file_names.end(),
              [](const std::string& lhs, const std::string& rhs) {
                if (lhs.length() != rhs.length()) {
                  return lhs.length() < rhs.length();
                }
                return lhs <= rhs;
              });

    for (size_t i = 0; i < pcd_file_names.size(); ++i) {
      AINFO << "***************** Frame " << i << " ******************";
      AINFO << pcd_file_names[i];

      std::shared_ptr<LidarFrame> frame = LidarFramePool::Instance().Get();
      if (frame->cloud == nullptr) {
        frame->cloud = base::PointFCloudPool::Instance().Get();
      }
      if (frame->world_cloud == nullptr) {
        frame->world_cloud = base::PointDCloudPool::Instance().Get();
      }
      LoadPCLPCD(pcd_file_names[i], frame->cloud.get());
      // frame->lidar2world_pose is default identity matrix
      TransformCloud(frame->cloud, frame->lidar2world_pose, frame->world_cloud);
      AINFO << "Read point cloud from " << pcd_file_names[i]
            << " with cloud size: " << frame->cloud->size();

      GroundDetectorOptions ground_detector_options;
      if (!ground_detector_->Detect(ground_detector_options, frame.get())) {
        AERROR << "Ground detect error.";
        return false;
      }

      std::string output_path = FLAGS_output_path;
      SavePCD(i, frame.get(), output_path);
    }

    return true;
  }

  bool SavePCD(size_t frame_id, LidarFrame* frame, const std::string& path) {
    if (!cyber::common::EnsureDirectory(path)) {
      AERROR << "Failed to create: " << path;
      return false;
    }

    std::string file_path = absl::StrCat(path, "/", frame_id, ".pcd");
    WritePcdByLabel(file_path, *(frame->cloud), LidarPointLabel::GROUND);

    return true;
  }

 protected:
  std::unique_ptr<BaseGroundDetector> ground_detector_;
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = 1;
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  apollo::perception::lidar::OfflinePointcloudGroundDetection test;
  if (!test.Init()) {
    AINFO << "Failed to setup OfflinePointcloudGroundDetection";
    return -1;
  }
  return test.Run() ? 0 : -1;
}
