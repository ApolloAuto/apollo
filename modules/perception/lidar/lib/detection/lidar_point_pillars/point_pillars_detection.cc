/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
#include <cuda_runtime_api.h>
#include <vector>

#include "cyber/common/log.h"

#include "modules/perception/base/object_pool_types.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lidar/common/lidar_timer.h"
#include "modules/perception/lidar/lib/detection/lidar_point_pillars/point_pillars_detection.h"

namespace apollo {
namespace perception {
namespace lidar {

using base::Object;
using base::PointD;
using base::PointF;

bool PointPillarsDetection::Init(const DetectionInitOptions& options) {
  point_pillars_ptr_.reset(
      new PointPillars(kReproduceResultMode, kScoreThreshold,
                       kNmsOverlapThreshold, FLAGS_pfe_onnx_file,
                       FLAGS_rpn_onnx_file));
  return true;
}

bool PointPillarsDetection::Detect(const DetectionOptions& options,
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

  Timer timer;

  if (cudaSetDevice(FLAGS_gpu_id) != cudaSuccess) {
    AERROR << "Failed to set device to gpu " << FLAGS_gpu_id;
    return false;
  }

  // transform point cloud into an array
  float* points_array = new float[original_cloud_->size() * 4];
  PclToArray(original_cloud_, points_array, kNormalizingFactor);

  // inference
  std::vector<float> out_detections;
  std::vector<int> out_labels;
  point_pillars_ptr_->DoInference(points_array, original_cloud_->size(),
                                  &out_detections, &out_labels);
  inference_time_ = timer.toc(true);

  // transfer output bounding boxs to objects
  GetObjects(&frame->segmented_objects, frame->lidar2world_pose,
             &out_detections, &out_labels);

  AINFO << "PointPillars: inference: " << inference_time_ << "\t"
        << "collect: " << collect_time_;
  return true;
}

void PointPillarsDetection::PclToArray(const base::PointFCloudPtr& pc_ptr,
                                       float* out_points_array,
                                       const float normalizing_factor) {
  for (size_t i = 0; i < pc_ptr->size(); ++i) {
    const auto& point = pc_ptr->at(i);
    out_points_array[i * 4 + 0] = point.x;
    out_points_array[i * 4 + 1] = point.y;
    out_points_array[i * 4 + 2] = point.z;
    out_points_array[i * 4 + 3] =
        static_cast<float>(point.intensity / normalizing_factor);
  }
}

void PointPillarsDetection::GetObjects(
    std::vector<std::shared_ptr<Object>>* objects, const Eigen::Affine3d& pose,
    std::vector<float>* detections, std::vector<int>* labels) {
  Timer timer;
  int num_objects = detections->size() / kOutputNumBoxFeature;

  objects->clear();
  base::ObjectPool::Instance().BatchGet(num_objects, objects);

  for (int i = 0; i < num_objects; ++i) {
    auto& object = objects->at(i);
    object->id = i;

    // read params of bounding box
    float x = detections->at(i * kOutputNumBoxFeature + 0);
    float y = detections->at(i * kOutputNumBoxFeature + 1);
    float z = detections->at(i * kOutputNumBoxFeature + 2);
    float dx = detections->at(i * kOutputNumBoxFeature + 4);
    float dy = detections->at(i * kOutputNumBoxFeature + 3);
    float dz = detections->at(i * kOutputNumBoxFeature + 5);
    float yaw = detections->at(i * kOutputNumBoxFeature + 6);
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
    for (float vx : std::vector<float>{dx/2, -dx/2}) {
      for (float vy : std::vector<float>{dy/2, -dy/2}) {
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
    object->lidar_supplement.raw_probs.back()[
        static_cast<int>(object->type)] = 1.0f;
    // copy to type
    object->type_probs.assign(object->lidar_supplement.raw_probs.back().begin(),
                              object->lidar_supplement.raw_probs.back().end());
  }

  collect_time_ = timer.toc(true);
}

// TODO(chenjiahao): update the base ObjectSubType with more fine-grained types
base::ObjectSubType PointPillarsDetection::GetObjectSubType(const int label) {
  switch (label) {
    case 0:
      return base::ObjectSubType::BUS;
    case 1:
      return base::ObjectSubType::CAR;
    case 2:  // construction vehicle
      return base::ObjectSubType::UNKNOWN_MOVABLE;
    case 3:  // trailer
      return base::ObjectSubType::UNKNOWN_MOVABLE;
    case 4:
      return base::ObjectSubType::TRUCK;
    case 5:  // barrier
      return base::ObjectSubType::UNKNOWN_UNMOVABLE;
    case 6:
      return base::ObjectSubType::CYCLIST;
    case 7:
      return base::ObjectSubType::MOTORCYCLIST;
    case 8:
      return base::ObjectSubType::PEDESTRIAN;
    case 9:
      return base::ObjectSubType::TRAFFICCONE;
    default:
      return base::ObjectSubType::UNKNOWN;
  }
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
