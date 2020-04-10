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
      new PointPillars(reproduce_result_mode_, score_threshold_,
                       nms_overlap_threshold_, FLAGS_pfe_onnx_file,
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
  point_pillars_ptr_->doInference(points_array, original_cloud_->size(),
                                  &out_detections);
  inference_time_ = timer.toc(true);

  // transfer output bounding boxs to objects
  GetObjects(&frame->segmented_objects, frame->lidar2world_pose,
             &out_detections);

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
    std::vector<float>* detections) {
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
    float dx2cos = dx * cosf(yaw) / 2;
    float dy2sin = dy * sinf(yaw) / 2;
    float dx2sin = dx * sinf(yaw) / 2;
    float dy2cos = dy * cosf(yaw) / 2;
    object->lidar_supplement.num_points_in_roi = 8;
    object->lidar_supplement.on_use = true;
    object->lidar_supplement.is_background = false;
    for (int j = 0; j < 2; ++j) {
      PointF point0, point1, point2, point3;
      float vz = z + (j == 0 ? 0 : dz);
      point0.x = x + dx2cos + dy2sin;
      point0.y = y + dx2sin - dy2cos;
      point0.z = vz;
      point1.x = x + dx2cos - dy2sin;
      point1.y = y + dx2sin + dy2cos;
      point1.z = vz;
      point2.x = x - dx2cos - dy2sin;
      point2.y = y - dx2sin + dy2cos;
      point2.z = vz;
      point3.x = x - dx2cos + dy2sin;
      point3.y = y - dx2sin - dy2cos;
      point3.z = vz;
      object->lidar_supplement.cloud.push_back(point0);
      object->lidar_supplement.cloud.push_back(point1);
      object->lidar_supplement.cloud.push_back(point2);
      object->lidar_supplement.cloud.push_back(point3);
    }
    for (auto& pt : object->lidar_supplement.cloud) {
      Eigen::Vector3d trans_point(pt.x, pt.y, pt.z);
      trans_point = pose * trans_point;
      PointD world_point;
      world_point.x = trans_point(0);
      world_point.y = trans_point(1);
      world_point.z = trans_point(2);
      object->lidar_supplement.cloud_world.push_back(world_point);
    }

    // classification (only detect vehicles so far)
    // TODO(chenjiahao): Fill object types completely
    object->lidar_supplement.raw_probs.push_back(std::vector<float>(
        static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE), 0.f));
    object->lidar_supplement.raw_classification_methods.push_back(Name());
    object->lidar_supplement.raw_probs
        .back()[static_cast<int>(base::ObjectType::VEHICLE)] = 1.0f;
    // copy to type
    object->type_probs.assign(object->lidar_supplement.raw_probs.back().begin(),
                              object->lidar_supplement.raw_probs.back().end());
    object->type = static_cast<base::ObjectType>(
        std::distance(object->type_probs.begin(),
                      std::max_element(object->type_probs.begin(),
                                       object->type_probs.end())));
  }

  collect_time_ = timer.toc(true);
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
