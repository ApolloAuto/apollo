/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/common/lidar/common/pcl_util.h"
#include "modules/perception/lidar_cpdet_detection/interface/base_cpdetector.h"
#include "modules/perception/common/lidar/common/object_builder.h"

DEFINE_string(pcd_path, "./pcd/", "pcd path");
DEFINE_string(output_path, "./output/", "output path");
DEFINE_string(detector_name, "CNNSegmentation", "detector name");
DEFINE_string(config_path, "perception/lidar_detection/data", "config path");
DEFINE_string(config_file, "cnnseg16_param.pb.txt", "config file");
DEFINE_string(sensor_name, "velodyne64", "sensor name");

namespace apollo {
namespace perception {
namespace lidar {

class OfflineLidarCPDetection {
 public:
  OfflineLidarCPDetection() = default;

  ~OfflineLidarCPDetection() = default;

  bool Init() {
    BaseCPDetector* detector =
        BaseCPDetectorRegisterer::GetInstanceByName(FLAGS_detector_name);
    CHECK_NOTNULL(detector);
    detector_.reset(detector);

    CPDetectorInitOptions detection_init_options;
    detection_init_options.sensor_name = FLAGS_sensor_name;
    detection_init_options.config_path = FLAGS_config_path;
    detection_init_options.config_file = FLAGS_config_file;
    ACHECK(detector_->Init(detection_init_options))
        << "lidar detector init error";

    ObjectBuilderInitOptions builder_init_options;
    builder_.reset(new ObjectBuilder);
    ACHECK(builder_->Init(builder_init_options));

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

      CPDetectorOptions detection_options;
      if (!detector_->Detect(detection_options, frame.get())) {
        AERROR << "Lidar detector detect error!";
        return false;
      }

      ObjectBuilderOptions builder_options;
      if (!builder_->Build(builder_options, frame.get())) {
        AERROR << "Lidar detector, object builder error.";
        return false;
      }

      std::string output_path = FLAGS_output_path;
      WriteObjectsForNewBenchmark(i, frame.get(), output_path);
    }

    return true;
  }

  bool WriteObjectsForNewBenchmark(size_t frame_id, LidarFrame* frame,
                                   const std::string& path) {
    if (!cyber::common::EnsureDirectory(path)) {
      AERROR << "Failed to create: " << path;
      return false;
    }
    std::ofstream fout;
    fout.open(absl::StrCat(path, "/", frame_id, ".txt"));
    fout << frame_id << " " << frame->segmented_objects.size() << std::endl;

    for (auto& object : frame->segmented_objects) {
      fout << "velodyne_64"
           << " " << object->id << " " << object->track_id << " "
           << object->lidar_supplement.is_background << " "
           << std::setprecision(8) << object->confidence << " ";

      std::string type = "unknow";
      if (object->type == base::ObjectType::UNKNOWN ||
          object->type == base::ObjectType::UNKNOWN_MOVABLE ||
          object->type == base::ObjectType::UNKNOWN_UNMOVABLE) {
        // type is unknow in this case
      } else if (object->type == base::ObjectType::PEDESTRIAN) {
        type = "pedestrian";
      } else if (object->type == base::ObjectType::VEHICLE) {
        type = "smallMot";
      } else if (object->type == base::ObjectType::BICYCLE) {
        type = "nonMot";
      }

      double yaw = atan2(object->direction(1), object->direction(0));
      auto& object_cloud = object->lidar_supplement.cloud;

      fout << type << " " << object->center(0) << " " << object->center(1)
           << " " << object->center(2) << " " << object->size(0) << " "
           << object->size(1) << " " << object->size(2) << " "
           << yaw /*yaw*/ << " " << 0 /*roll*/ << " " << 0 /*pitch*/ << " "
           << 0 /*truncated*/ << " " << 0 /*occluded*/ << " "
           << object->velocity(0) << " " << object->velocity(1) << " "
           << object->velocity(2) << " ";
      fout << object_cloud.size() << " ";
      for (size_t i = 0; i < object_cloud.size(); ++i) {
        auto& pt = object_cloud.at(i);
        fout << std::setprecision(8) << pt.x << " " << pt.y << " " << pt.z
             << " " << pt.intensity << " ";
      }
      // fout << 0/*#indices*/ << " ";
      // we save polygon here for debug
      int polygon_size = -static_cast<int>(object->polygon.size());
      fout << polygon_size /*negative polygon size*/ << " ";
      for (size_t i = 0; i < object->polygon.size(); ++i) {
        const auto& pt = object->polygon[i];
        fout << std::setprecision(8) << pt.x << " " << pt.y << " " << pt.z
             << " ";
      }
      fout << std::endl;
    }

    fout.close();
    return true;
  }

 protected:
  std::unique_ptr<BaseCPDetector> detector_;
  std::unique_ptr<ObjectBuilder> builder_;
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = 1;
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  // Todo(zero): core dump when rt_net is destructed!
  apollo::perception::lidar::OfflineLidarCPDetection test;
  if (!test.Init()) {
    AINFO << "Failed to setup OfflineLidarCPDetection";
    return -1;
  }
  return test.Run() ? 0 : -1;
}
