/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/lidar/lib/detector/mask_pillars_detection/mask_pillars_detection.h"

#include <algorithm>
#include <numeric>
#include <random>

#include "cyber/common/log.h"
#include "modules/perception/base/object_pool_types.h"
#include "modules/perception/base/point_cloud_util.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lidar/common/lidar_timer.h"
#include "modules/perception/lidar/common/pcl_util.h"
#include "modules/perception/lidar/lib/detector/point_pillars_detection/params.h"

namespace apollo {
namespace perception {
namespace lidar {

using base::Object;
using base::PointD;
using base::PointF;

MaskPillarsDetection::MaskPillarsDetection()
    : x_min_(Params::kMinXRange),
      x_max_(Params::kMaxXRange),
      y_min_(Params::kMinYRange),
      y_max_(Params::kMaxYRange),
      z_min_(Params::kMinZRange),
      z_max_(Params::kMaxZRange) {
  if (FLAGS_enable_ground_removal) {
    z_min_ = std::max(z_min_, static_cast<float>(FLAGS_ground_removal_height));
  }
}

// TODO(chenjiahao):
//  specify score threshold and nms over lap threshold for each class.
bool MaskPillarsDetection::Init(const LidarDetectorInitOptions& options) {
  point_pillars_ptr_.reset(
      new PointPillars(FLAGS_reproduce_result_mode, FLAGS_score_threshold,
                       FLAGS_nms_overlap_threshold, FLAGS_mask_pfe_torch_file,
                       FLAGS_mask_scattered_torch_file,
                       FLAGS_mask_backbone_torch_file,
                       FLAGS_mask_fpn_torch_file,
                       FLAGS_mask_bbox_head_torch_file));
  return true;
}

bool MaskPillarsDetection::Init(const StageConfig& stage_config) {
  if (!Initialize(stage_config)) {
    return false;
  }

  point_pillars_ptr_.reset(
      new PointPillars(FLAGS_reproduce_result_mode,
                       FLAGS_score_threshold,
                       FLAGS_nms_overlap_threshold,
                       FLAGS_mask_pfe_torch_file,
                       FLAGS_mask_scattered_torch_file,
                       FLAGS_mask_backbone_torch_file,
                       FLAGS_mask_fpn_torch_file,
                       FLAGS_mask_bbox_head_torch_file));
  return true;
}

bool MaskPillarsDetection::Process(DataFrame* data_frame) {
  if (data_frame == nullptr)
    return false;

  LidarFrame* lidar_frame = data_frame->lidar_frame;
  if (lidar_frame == nullptr)
    return false;

  LidarDetectorOptions options;
  bool res = Detect(options, lidar_frame);
  return res;
}

bool MaskPillarsDetection::Detect(const LidarDetectorOptions& options,
                                   LidarFrame* frame) {
  // check input
  if (frame == nullptr) {
    AERROR << "Input null frame ptr.";
    return false;
  }
  if (frame->cloud == nullptr) {
    AERROR << "Input null frame cloud.";
    return false;
  }
  if (frame->cloud->size() == 0) {
    AERROR << "Input none points.";
    return false;
  }

  // record input cloud and lidar frame
  original_cloud_ = frame->cloud;
  original_world_cloud_ = frame->world_cloud;
  lidar_frame_ref_ = frame;

  // check output
  frame->segmented_objects.clear();

  if (cudaSetDevice(FLAGS_gpu_id) != cudaSuccess) {
    AERROR << "Failed to set device to gpu " << FLAGS_gpu_id;
    return false;
  }

  Timer timer;

  int num_points;
  cur_cloud_ptr_ = std::shared_ptr<base::PointFCloud>(
      new base::PointFCloud(*original_cloud_));

  // down sample the point cloud through filtering beams
  if (FLAGS_enable_downsample_beams) {
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

  // down sample the point cloud through filtering voxel grid
  if (FLAGS_enable_downsample_pointcloud) {
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
  float* points_array = new float[num_points * FLAGS_num_point_feature]();
  CloudToArray(cur_cloud_ptr_, points_array, FLAGS_normalizing_factor);
  cloud_to_array_time_ = timer.toc(true);

  // inference
  std::vector<float> out_detections;
  std::vector<int> out_labels;
  point_pillars_ptr_->DoInference(points_array, num_points, &out_detections,
                                  &out_labels);
  inference_time_ = timer.toc(true);

  // transfer output bounding boxes to objects
  GetObjects(&frame->segmented_objects, frame->lidar2world_pose,
             &out_detections, &out_labels);
  collect_time_ = timer.toc(true);

  AINFO << "PointPillars: "
        << "\n"
        << "down sample: " << downsample_time_ << "\t"
        << "fuse: " << fuse_time_ << "\t"
        << "shuffle: " << shuffle_time_ << "\t"
        << "cloud_to_array: " << cloud_to_array_time_ << "\t"
        << "inference: " << inference_time_ << "\t"
        << "collect: " << collect_time_;

  delete[] points_array;
  return true;
}

void MaskPillarsDetection::CloudToArray(const base::PointFCloudPtr& pc_ptr,
                                         float* out_points_array,
                                         const float normalizing_factor) {
  for (size_t i = 0; i < pc_ptr->size(); ++i) {
    const auto& point = pc_ptr->at(i);
    float x = point.x;
    float y = point.y;
    float z = point.z;
    float intensity = point.intensity;
    if (z < z_min_ || z > z_max_ || y < y_min_ || y > y_max_ || x < x_min_ ||
        x > x_max_) {
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

void MaskPillarsDetection::FuseCloud(
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

std::vector<int> MaskPillarsDetection::GenerateIndices(int start_index,
                                                        int size,
                                                        bool shuffle) {
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

void MaskPillarsDetection::GetObjects(
    std::vector<std::shared_ptr<Object>>* objects, const Eigen::Affine3d& pose,
    std::vector<float>* detections, std::vector<int>* labels) {
  int num_objects = detections->size() / FLAGS_num_output_box_feature;

  objects->clear();
  base::ObjectPool::Instance().BatchGet(num_objects, objects);

  for (int i = 0; i < num_objects; ++i) {
    auto& object = objects->at(i);
    object->id = i;

    // read params of bounding box
    float x = detections->at(i * FLAGS_num_output_box_feature + 0);
    float y = detections->at(i * FLAGS_num_output_box_feature + 1);
    float z = detections->at(i * FLAGS_num_output_box_feature + 2);
    float dx = detections->at(i * FLAGS_num_output_box_feature + 4);
    float dy = detections->at(i * FLAGS_num_output_box_feature + 3);
    float dz = detections->at(i * FLAGS_num_output_box_feature + 5);
    float yaw = detections->at(i * FLAGS_num_output_box_feature + 6);
    yaw += M_PI / 2;
    yaw = std::atan2(sinf(yaw), cosf(yaw));
    yaw = -yaw;

    // directions
    object->theta = yaw;
    object->direction[0] = cosf(yaw);
    object->direction[1] = sinf(yaw);
    object->direction[2] = 0;
    object->lidar_supplement.is_orientation_ready = true;

    // compute vertexes of bounding box and transform to world coordinate
    object->lidar_supplement.num_points_in_roi = 8;
    object->lidar_supplement.on_use = true;
    object->lidar_supplement.is_background = false;
    float roll = 0, pitch = 0;
    Eigen::Quaternionf quater =
        Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
        Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f translation(x, y, z);
    Eigen::Affine3f affine3f = translation * quater.toRotationMatrix();
    for (float vx : std::vector<float>{dx / 2, -dx / 2}) {
      for (float vy : std::vector<float>{dy / 2, -dy / 2}) {
        for (float vz : std::vector<float>{0, dz}) {
          Eigen::Vector3f v3f(vx, vy, vz);
          v3f = affine3f * v3f;
          PointF point;
          point.x = v3f.x();
          point.y = v3f.y();
          point.z = v3f.z();
          object->lidar_supplement.cloud.push_back(point);

          Eigen::Vector3d trans_point(point.x, point.y, point.z);
          trans_point = pose * trans_point;
          PointD world_point;
          world_point.x = trans_point(0);
          world_point.y = trans_point(1);
          world_point.z = trans_point(2);
          object->lidar_supplement.cloud_world.push_back(world_point);
        }
      }
    }

    // classification
    object->lidar_supplement.raw_probs.push_back(std::vector<float>(
        static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE), 0.f));
    object->lidar_supplement.raw_classification_methods.push_back(Name());
    object->sub_type = GetObjectSubType(labels->at(i));
    object->type = base::kSubType2TypeMap.at(object->sub_type);
    object->lidar_supplement.raw_probs.back()[static_cast<int>(object->type)] =
        1.0f;
    // copy to type
    object->type_probs.assign(object->lidar_supplement.raw_probs.back().begin(),
                              object->lidar_supplement.raw_probs.back().end());
  }
}

// TODO(all): update the base ObjectSubType with more fine-grained types
// TODO(chenjiahao): move types into an array in the same order as offline
base::ObjectSubType MaskPillarsDetection::GetObjectSubType(const int label) {
  switch (label) {
    case 0:
      return base::ObjectSubType::CAR;
    case 1:
      return base::ObjectSubType::PEDESTRIAN;
    case 2:  // construction vehicle
      return base::ObjectSubType::CYCLIST;
    default:
      return base::ObjectSubType::UNKNOWN;
  }
}

PERCEPTION_REGISTER_LIDARDETECTOR(MaskPillarsDetection);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
