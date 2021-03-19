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

#include "Eigen/Dense"
#include "gflags/gflags.h"
#include "pcl/io/pcd_io.h"
#include "pcl/kdtree/impl/kdtree_flann.hpp"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/kdtree_flann.h"

#include "absl/strings/str_cat.h"
#include "cyber/common/file.h"
#include "modules/perception/base/object.h"
#include "modules/perception/base/object_types.h"
#include "modules/perception/base/point_cloud.h"
#include "modules/perception/common/io/io_util.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/common/point_cloud_processing/common.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/app/lidar_obstacle_segmentation.h"
#include "modules/perception/lidar/app/lidar_obstacle_tracking.h"
#include "modules/perception/lidar/common/lidar_frame.h"
#include "modules/perception/lidar/common/lidar_frame_pool.h"
#include "modules/perception/lidar/common/lidar_log.h"
#include "modules/perception/lidar/common/pcl_util.h"

DEFINE_string(pcd_path, "./pcd/", "pcd path");
DEFINE_string(pose_path, "", "pose path");
DEFINE_string(output_path, "./output/", "output path");
DEFINE_bool(enable_tracking, false, "option to enable tracking");
DEFINE_double(min_life_time, -1.0, "minimum track time for output");
DEFINE_bool(use_hdmap, false, "option to enable using hdmap");
DEFINE_bool(use_tracking_info, false, "option to use tracking info");
DEFINE_string(sensor_name, "velodyne64", "sensor name");

namespace apollo {
namespace perception {
namespace lidar {

using cyber::common::GetFileName;

class OfflineLidarObstaclePerception {
 public:
  OfflineLidarObstaclePerception() = default;

  ~OfflineLidarObstaclePerception() = default;

  bool setup() {
    FLAGS_config_manager_path = "./conf";
    if (!lib::ConfigManager::Instance()->Init()) {
      AERROR << "Failed to init ConfigManage.";
      return false;
    }
    lidar_segmentation_.reset(new LidarObstacleSegmentation);
    if (lidar_segmentation_ == nullptr) {
      AERROR << "Failed to get LidarObstacleSegmentation instance.";
      return false;
    }
    segment_init_options_.enable_hdmap_input = FLAGS_use_hdmap;
    segment_init_options_.sensor_name = FLAGS_sensor_name;
    if (!lidar_segmentation_->Init(segment_init_options_)) {
      AINFO << "Failed to init LidarObstacleSegmentation.";
      return false;
    }
    lidar_tracking_.reset(new LidarObstacleTracking);
    if (lidar_tracking_ == nullptr) {
      AERROR << "Failed to get LidarObstacleTracking instance.";
      return false;
    }
    tracking_init_options_.sensor_name = FLAGS_sensor_name;
    if (!lidar_tracking_->Init(tracking_init_options_)) {
      AINFO << "Failed to init LidarObstacleSegmentation.";
      return false;
    }

    if (!common::SensorManager::Instance()->GetSensorInfo(FLAGS_sensor_name,
                                                          &sensor_info_)) {
      AERROR << "Failed to get sensor info, sensor name: " << FLAGS_sensor_name;
      return false;
    }

    ADEBUG << "Sensor_name: " << sensor_info_.name;
    return true;
  }

  bool run() {
    double timestamp = 0.f;
    std::string pcd_folder = FLAGS_pcd_path;
    std::string pose_folder = FLAGS_pose_path;
    std::string output_path = FLAGS_output_path;
    std::vector<std::string> pcd_file_names;
    if (!common::GetFileList(pcd_folder, ".pcd", &pcd_file_names)) {
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
      const std::string file_name = GetFileName(pcd_file_names[i]);
      frame_ = LidarFramePool::Instance().Get();
      frame_->sensor_info = sensor_info_;
      frame_->reserve = file_name;
      if (frame_->cloud == nullptr) {
        frame_->cloud = base::PointFCloudPool::Instance().Get();
      }
      LoadPCLPCD(pcd_folder + "/" + file_name + ".pcd", frame_->cloud.get());
      AINFO << "Read point cloud from " << pcd_file_names[i]
            << " with cloud size: " << frame_->cloud->size();
      if (pose_folder != "") {
        std::string pose_file_name = pose_folder + "/" + file_name + ".pose";
        AINFO << "Pose file: " << pose_file_name;
        if (!apollo::cyber::common::PathExists(pose_file_name)) {
          pose_file_name = pose_folder + "/" + file_name + ".pcd.pose";
        }
        int idt = 0;
        if (common::ReadPoseFile(pose_file_name, &frame_->lidar2world_pose,
                                 &idt, &timestamp)) {
          AINFO << "[timestamp]: " << std::setprecision(16) << timestamp;
          frame_->timestamp = timestamp;
        } else {
          AINFO << "Failed to load pose, disable tracking pipeline.";
          FLAGS_enable_tracking = false;
        }
      }
      // TODO(shitingmin) undo timestamp.
      LidarProcessResult segment_result =
          lidar_segmentation_->Process(segment_options_, frame_.get());
      if (segment_result.error_code != LidarErrorCode::Succeed) {
        AINFO << segment_result.log;
        return false;
      }
      if (FLAGS_enable_tracking) {
        AINFO << "Enable tracking.";
        LidarProcessResult tracking_result =
            lidar_tracking_->Process(tracking_options_, frame_.get());
        if (tracking_result.error_code != LidarErrorCode::Succeed) {
          AINFO << tracking_result.log;
          return false;
        }
        if (FLAGS_use_tracking_info) {
          auto& objects = frame_->segmented_objects;
          auto& result_objects = frame_->tracked_objects;
          std::sort(objects.begin(), objects.end(),
                    [](const base::ObjectPtr& lhs, const base::ObjectPtr& rhs) {
                      return lhs->id < rhs->id;
                    });
          std::sort(result_objects.begin(), result_objects.end(),
                    [](const base::ObjectPtr& lhs, const base::ObjectPtr& rhs) {
                      return lhs->id < rhs->id;
                    });
          for (std::size_t j = 0; j < objects.size(); ++j) {
            ACHECK(objects[j]->id == result_objects[j]->id);
            objects[j]->track_id = result_objects[j]->track_id;
            objects[j]->tracking_time = result_objects[j]->tracking_time;
            objects[j]->center =
                frame_->lidar2world_pose.inverse() * result_objects[j]->center;
            Eigen::Vector3d direction =
                result_objects[j]->direction.cast<double>();
            direction =
                frame_->lidar2world_pose.linear().transpose() * direction;
            objects[j]->direction = direction.cast<float>();
            objects[j]->size = result_objects[j]->size;
            auto velocity = frame_->lidar2world_pose.linear().transpose() *
                            result_objects[j]->velocity.cast<double>();
            objects[j]->velocity = velocity.cast<float>();

            objects[j]->type = result_objects[j]->type;
            objects[j]->type_probs = result_objects[j]->type_probs;
            objects[j]->polygon = result_objects[j]->polygon;
            common::TransformPointCloud(result_objects[j]->polygon,
                                        frame_->lidar2world_pose.inverse(),
                                        &objects[j]->polygon);
          }
        }
      }

      std::vector<base::ObjectPtr>& result_objects = frame_->segmented_objects;
      std::vector<base::ObjectPtr> filtered_objects;
      for (auto& object : result_objects) {
        if (object->tracking_time >= FLAGS_min_life_time) {
          filtered_objects.push_back(object);
        }
      }
      if (!WriteObjectsForNewBenchmark(
              i, filtered_objects,
              absl::StrCat(output_path, "/", file_name, ".txt"))) {
        return false;
      }
    }

    return true;
  }

  bool WriteObjectsForNewBenchmark(size_t frame_id,
                                   const std::vector<base::ObjectPtr>& objects,
                                   const std::string& path) {
    std::ofstream fout(path);
    if (!fout.is_open()) {
      AERROR << "Failed to open: " << path;
      return false;
    }
    fout << frame_id << " " << objects.size() << std::endl;
    for (auto& object : objects) {
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
    // We save road boundary and road/junction polygon here for debug
    if (frame_->hdmap_struct != nullptr) {
      // Transform to local frame, note since missing valid z value, the local
      // coordinates may not be accurate, only for visualization purpose
      double z = frame_->lidar2world_pose(2, 3) - 1.7;
      Eigen::Affine3d world2lidar_pose = frame_->lidar2world_pose.inverse();
      auto write_transformed_polygon = [&](const base::PolygonDType& poly) {
        Eigen::Vector3d point;
        fout << poly.size() << " ";
        for (size_t i = 0; i < poly.size(); ++i) {
          const auto& pt = poly[i];
          point << pt.x, pt.y, z;
          point = world2lidar_pose * point;
          fout << std::setprecision(8) << point(0) << " " << point(1) << " "
               << point(2) << " ";
        }
        fout << std::endl;
      };
      // 1. Write road boundary
      fout << frame_->hdmap_struct->road_boundary.size() << std::endl;
      for (auto& pair : frame_->hdmap_struct->road_boundary) {
        write_transformed_polygon(pair.left_boundary);
        write_transformed_polygon(pair.right_boundary);
      }
      // 2. Write road/junction polygon
      fout << frame_->hdmap_struct->road_polygons.size() +
                  frame_->hdmap_struct->junction_polygons.size()
           << std::endl;
      for (auto& poly : frame_->hdmap_struct->road_polygons) {
        write_transformed_polygon(poly);
      }
      for (auto& poly : frame_->hdmap_struct->junction_polygons) {
        write_transformed_polygon(poly);
      }
      // 3. Save points label
      fout << frame_->cloud->size() << " ";
      for (size_t i = 0; i < frame_->cloud->size(); ++i) {
        fout << static_cast<int>(frame_->cloud->points_label(i)) << " ";
      }
      fout << std::endl;
    }

    fout.close();
    return true;
  }

 protected:
  std::string output_dir_;
  std::shared_ptr<LidarFrame> frame_;
  LidarObstacleSegmentationInitOptions segment_init_options_;
  LidarObstacleSegmentationOptions segment_options_;
  LidarObstacleTrackingInitOptions tracking_init_options_;
  LidarObstacleTrackingOptions tracking_options_;
  std::unique_ptr<LidarObstacleSegmentation> lidar_segmentation_;
  std::unique_ptr<LidarObstacleTracking> lidar_tracking_;
  base::SensorInfo sensor_info_;
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = 1;
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  apollo::perception::lidar::OfflineLidarObstaclePerception test;
  if (!test.setup()) {
    AINFO << "Failed to setup OfflineLidarObstaclePerception";
    return -1;
  }
  return test.run() ? 0 : -1;
}
