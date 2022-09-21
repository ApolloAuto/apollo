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

#include "modules/perception/lidar/lib/pointcloud_detection_preprocessor/pointcloud_down_sample/pointcloud_down_sample.h"

#include <random>

#include "modules/perception/base/point_cloud_util.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lidar/common/lidar_timer.h"
#include "modules/perception/lidar/common/pcl_util.h"

#include "modules/perception/pipeline/proto/plugin/pointcloud_downsample_config.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

PointCloudDownSample::PointCloudDownSample(const PluginConfig& plugin_config) {
  Init(plugin_config);
}

bool PointCloudDownSample::Init(const PluginConfig& plugin_config) {
  ACHECK(plugin_config.has_pointcloud_downsample_config());
  auto config = plugin_config.pointcloud_downsample_config();
  enable_downsample_pointcloud_ = config.enable_downsample_pointcloud();
  enable_downsample_beams_ = config.enable_downsample_beams();
  x_min_range_ = config.x_min_range();
  x_max_range_ = config.x_max_range();
  y_min_range_ = config.y_min_range();
  y_max_range_ = config.y_max_range();
  z_min_range_ = config.z_min_range();
  z_max_range_ = config.z_max_range();
  return true;
}

bool PointCloudDownSample::Process(DataFrame* data_frame,
                                   std::vector<float>* points_array,
                                   int* num_points_result) {
  auto lidar_frame = data_frame->lidar_frame;
  if (!DownSample(lidar_frame, points_array, num_points_result)) {
    return false;
  }
  return true;
}

bool PointCloudDownSample::DownSample(LidarFrame* lidar_frame,
                                      std::vector<float>* points_array,
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

  int num_points;
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
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZI>());
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
           lidar_frame->timestamp -
                   prev_world_clouds_.front()->get_timestamp() >
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
      base::PointD world_point;
      world_point.x = trans_point(0);
      world_point.y = trans_point(1);
      world_point.z = trans_point(2);
      world_point.intensity = pt.intensity;
      cur_world_cloud_ptr->push_back(world_point);
    }
    cur_world_cloud_ptr->set_timestamp(lidar_frame->timestamp);

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
  points_array->resize(num_points * FLAGS_num_point_feature);
  CloudToArray(cur_cloud_ptr_, points_array->data(), FLAGS_normalizing_factor);
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

void PointCloudDownSample::CloudToArray(const base::PointFCloudPtr& pc_ptr,
                                        float* out_points_array,
                                        const float normalizing_factor) {
  for (size_t i = 0; i < pc_ptr->size(); ++i) {
    const auto& point = pc_ptr->at(i);
    float x = point.x;
    float y = point.y;
    float z = point.z;
    float intensity = point.intensity;
    if (z < z_min_range_ || z > z_max_range_ || y < y_min_range_ ||
        y > y_max_range_ || x < x_min_range_ || x > x_max_range_) {
      continue;
    }
    out_points_array[i * FLAGS_num_point_feature + 0] = x;
    out_points_array[i * FLAGS_num_point_feature + 1] = y;
    out_points_array[i * FLAGS_num_point_feature + 2] = z;
    out_points_array[i * FLAGS_num_point_feature + 3] =
        intensity / normalizing_factor;
    // delta of timestamp between prev and cur frames
    out_points_array[i * FLAGS_num_point_feature + 4] =
        static_cast<float>(pc_ptr->points_timestamp(i));
  }
}

std::vector<int> PointCloudDownSample::GenerateIndices(int start_index,
                                                       int size, bool shuffle) {
  // create a range number array
  std::vector<int> indices(size);
  std::iota(indices.begin(), indices.end(), start_index);

  // shuffle the index array
  if (shuffle) {
    unsigned seed = 0;
    std::shuffle(indices.begin(), indices.end(),
                 std::default_random_engine(seed));
  }
  return indices;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
