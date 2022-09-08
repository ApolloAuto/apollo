/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/lidar/task/point_cloud_down_sample.h"

#include "modules/perception/base/point_cloud_util.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lidar/common/lidar_timer.h"
#include "modules/perception/lidar/common/pcl_util.h"

namespace apollo {
namespace perception {
namespace lidar {

bool PointCloudDownSample::Init(const TaskConfig& task_config) {
  ACHECK(task_config.has_pointcloud_down_sample());
  enable_downsample_pointcloud_ = task_config.enable_downsample_pointcloud();
  enable_downsample_beams_ = task_config.enable_downsample_beams();
  return true;
}

bool PointCloudDownSample::Process(DataFrame* data_frame, float* point_array) {
  if (data_frame == nullptr) {
    AERROR << "Input null data_frame ptr.";
    return false;
  }
  if (point_array == nullptr) {
    AERROR << "Input null point_array ptr.";
    return false;
  }

  auto lidar_frame = data_frame->lidar_frame;
  DownSample(data_frame, point_array);
}

bool PointCloudDownSample::DownSample(LidarFrame* lidar_frame,
                                      float* points_array,
                                      int* num_points_result) {
  if (lidar_frame->cloud == nullptr) {
    AERROR << "Input null frame cloud.";
    return false;
  }
  if (lidar_frame->cloud->size() == 0) {
    AERROR << "Input none points.";
    return false;
  }

  // record input cloud and lidar frame
  original_cloud_ = lidar_frame->cloud;
  original_world_cloud_ = lidar_frame->world_cloud;
  lidar_frame_ref_ = lidar_frame;

  if (cudaSetDevice(FLAGS_gpu_id) != cudaSuccess) {
    AERROR << "Failed to set device to gpu " << FLAGS_gpu_id;
    return false;
  }

  Timer timer;

  cur_cloud_ptr_ = std::shared_ptr<base::PointFCloud>(
      new base::PointFCloud(*original_cloud_));

  // down sample the point cloud through filtering beams
  if (enable_downsample_beams_) {
    base::PointFCloudPtr downsample_beams_cloud_ptr(new base::PointFCloud());
    if (DownSamplePointCloudBeams(original_cloud_, downsample_beams_cloud_ptr,
                                  FLAGS_downsample_beams_factor)) {
      cur_cloud_ptr_ = downsample_beams_cloud_ptr;
    } else {
      AWARN << "Down-sample beams factor must be >= 1. Cancel down-sampling."
               " Current factor: "
            << FLAGS_downsample_beams_factor;
    }
  }

  if (enable_downsample_pointcloud_) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZI>());
    TransformToPCLXYZI(*cur_cloud_ptr_, pcl_cloud_ptr);
    DownSampleCloudByVoxelGrid(
        pcl_cloud_ptr, filtered_cloud_ptr, FLAGS_downsample_voxel_size_x,
        FLAGS_downsample_voxel_size_y, FLAGS_downsample_voxel_size_z);

    // transform pcl point cloud to apollo point cloud
    base::PointFCloudPtr downsample_voxel_cloud_ptr(new base::PointFCloud());
    TransformFromPCLXYZI(filtered_cloud_ptr, downsample_voxel_cloud_ptr);
    cur_cloud_ptr_ = downsample_voxel_cloud_ptr;
  }

  downsample_time_ = timer.toc(true);
  num_points = cur_cloud_ptr_->size();
  AINFO << "num points before fusing: " << num_points;

  // fuse clouds of preceding frames with current cloud
  cur_cloud_ptr_->mutable_points_timestamp()->assign(cur_cloud_ptr_->size(),
                                                     0.0);
  if (FLAGS_enable_fuse_frames && FLAGS_num_fuse_frames > 1) {
    // before fusing
    while (!prev_world_clouds_.empty() &&
           frame->timestamp - prev_world_clouds_.front()->get_timestamp() >
               FLAGS_fuse_time_interval) {
      prev_world_clouds_.pop_front();
    }
    // transform current cloud to world coordinate and save to a new ptr
    base::PointDCloudPtr cur_world_cloud_ptr =
        std::make_shared<base::PointDCloud>();
    for (size_t i = 0; i < cur_cloud_ptr_->size(); ++i) {
      auto& pt = cur_cloud_ptr_->at(i);
      Eigen::Vector3d trans_point(pt.x, pt.y, pt.z);
      trans_point = lidar_frame_ref_->lidar2world_pose * trans_point;
      PointD world_point;
      world_point.x = trans_point(0);
      world_point.y = trans_point(1);
      world_point.z = trans_point(2);
      world_point.intensity = pt.intensity;
      cur_world_cloud_ptr->push_back(world_point);
    }
    cur_world_cloud_ptr->set_timestamp(frame->timestamp);

    // fusing clouds
    for (auto& prev_world_cloud_ptr : prev_world_clouds_) {
      num_points += prev_world_cloud_ptr->size();
    }
    FuseCloud(cur_cloud_ptr_, prev_world_clouds_);

    // after fusing
    while (static_cast<int>(prev_world_clouds_.size()) >=
           FLAGS_num_fuse_frames - 1) {
      prev_world_clouds_.pop_front();
    }
    prev_world_clouds_.emplace_back(cur_world_cloud_ptr);
  }
  AINFO << "num points after fusing: " << num_points;
  fuse_time_ = timer.toc(true);

  // shuffle points and cut off
  if (FLAGS_enable_shuffle_points) {
    num_points = std::min(num_points, FLAGS_max_num_points);
    std::vector<int> point_indices = GenerateIndices(0, num_points, true);
    base::PointFCloudPtr shuffle_cloud_ptr(
        new base::PointFCloud(*cur_cloud_ptr_, point_indices));
    cur_cloud_ptr_ = shuffle_cloud_ptr;
  }
  shuffle_time_ = timer.toc(true);

  // point cloud to array
  // float* points_array = new float[num_points * FLAGS_num_point_feature]();
  points_array->resize(num_points * FLAGS_num_point_feature)
      CloudToArray(cur_cloud_ptr_, points_array, FLAGS_normalizing_factor);
  *num_points_result = num_points;
  cloud_to_array_time_ = timer.toc(true);

  return true;
}

void PointCloudDownSample::FuseCloud(
    const base::PointFCloudPtr& out_cloud_ptr,
    const std::deque<base::PointDCloudPtr>& fuse_clouds) {
  for (auto iter = fuse_clouds.rbegin(); iter != fuse_clouds.rend(); ++iter) {
    double delta_t = lidar_frame_ref_->timestamp - (*iter)->get_timestamp();
    // transform prev world point cloud to current sensor's coordinates
    for (size_t i = 0; i < (*iter)->size(); ++i) {
      auto& point = (*iter)->at(i);
      Eigen::Vector3d trans_point(point.x, point.y, point.z);
      trans_point = lidar_frame_ref_->lidar2world_pose.inverse() * trans_point;
      base::PointF pt;
      pt.x = static_cast<float>(trans_point(0));
      pt.y = static_cast<float>(trans_point(1));
      pt.z = static_cast<float>(trans_point(2));
      pt.intensity = static_cast<float>(point.intensity);
      // delta of time between current and prev frame
      out_cloud_ptr->push_back(pt, delta_t);
    }
  }
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo