/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/radar4d_detection/lib/detector/point_pillars_detection/point_pillars_detection.h"

#include <algorithm>
#include <numeric>
#include <random>

#include <cuda_runtime_api.h>

#include "cyber/common/log.h"
#include "cyber/common/file.h"
#include "modules/perception/common/util.h"
#include "modules/perception/common/base/object_pool_types.h"
#include "modules/perception/common/base/point_cloud_util.h"
#include "modules/perception/common/radar/common/radar_timer.h"
#include "modules/perception/radar4d_detection/lib/detector/point_pillars_detection/params.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

namespace apollo {
namespace perception {
namespace radar4d {

using base::Object;
using base::RadarPointD;
using base::RadarPointF;

Radar4dDetection::Radar4dDetection()
    : x_min_(Params::kMinXRange),
      x_max_(Params::kMaxXRange),
      y_min_(Params::kMinYRange),
      y_max_(Params::kMaxYRange),
      z_min_(Params::kMinZRange),
      z_max_(Params::kMaxZRange) {}

bool Radar4dDetection::Init(const DetectorInitOptions& options) {
  // Init config
  std::string config_file =
      GetConfigFile(options.config_path, options.config_file);
  ACHECK(apollo::cyber::common::GetProtoFromFile(config_file, &param_));

  // Model Info
  const auto &model_info = param_.info();
  std::string model_path = GetModelPath(model_info.name());
  std::string weight_file =
      GetModelFile(model_path, model_info.weight_file().file());

  // Init pointpillars
  point_pillars_ptr_.reset(
      new PointPillars(param_.preprocess().reproduce_result_mode(),
                       param_.postprocess().score_threshold(),
                       param_.postprocess().nms_overlap_threshold(),
                       model_info.framework(),
                       weight_file));
  AINFO << "Radar4dDetection init is ok!";

  return true;
}

bool Radar4dDetection::Detect(RadarFrame* frame,
            const DetectorOptions& options) {
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

  // record input cloud and radar frame
  original_cloud_ = frame->cloud;
  original_world_cloud_ = frame->world_cloud;
  radar_frame_ref_ = frame;

  // check output
  frame->segmented_objects.clear();

  // check gpu
  if (cudaSetDevice(param_.preprocess().gpu_id()) != cudaSuccess) {
    AERROR << "Failed to set device to gpu " << param_.preprocess().gpu_id();
    return false;
  }

  Timer timer;

  int num_points;
  cur_cloud_ptr_ = std::shared_ptr<base::RadarPointFCloud>(
      new base::RadarPointFCloud(*original_cloud_));

  num_points = cur_cloud_ptr_->size();
  AINFO << "num points before fusing: " << num_points;

  // fuse clouds of preceding frames with current cloud
  // before fusing, filter the older point cloud
  while (!prev_world_clouds_.empty() &&
          frame->timestamp - prev_world_clouds_.front()->get_timestamp() >
              param_.preprocess().fuse_time_interval()) {
    prev_world_clouds_.pop_front();
  }
  // transform current cloud to world coordinate and save to a new ptr
  base::RadarPointDCloudPtr cur_world_cloud_ptr =
      std::make_shared<base::RadarPointDCloud>();
  for (size_t i = 0; i < cur_cloud_ptr_->size(); ++i) {
    auto& pt = cur_cloud_ptr_->at(i);
    Eigen::Vector3d trans_point(pt.x, pt.y, pt.z);
    trans_point = radar_frame_ref_->radar2world_pose * trans_point;
    RadarPointD world_point;
    world_point.x = trans_point(0);
    world_point.y = trans_point(1);
    world_point.z = trans_point(2);
    world_point.rcs = pt.rcs;
    world_point.velocity = pt.velocity;
    world_point.comp_vel = pt.comp_vel;
    cur_world_cloud_ptr->push_back(world_point);
  }
  cur_world_cloud_ptr->set_timestamp(frame->timestamp);

  // fusing clouds, and convert point cloud to array
  for (auto& prev_world_cloud_ptr : prev_world_clouds_) {
    num_points += prev_world_cloud_ptr->size();
  }

  float* points_array =
    new float[num_points * param_.preprocess().num_point_feature()]();

  FuseCloud(cur_cloud_ptr_,
    prev_world_clouds_,
    points_array,
    param_.preprocess().normalizing_factor());
  cloud_to_array_time_ = timer.toc(true);

  // output the fused point cloud to pcd file for visualization
  if (0) {
    pcl::PointCloud<pcl::PointXYZI> cloud;

    cloud.width = 1;
    cloud.height = cur_cloud_ptr_->size();
    cloud.is_dense = false;
    cloud.points.reserve(cur_cloud_ptr_->size());

    for (size_t i = 0; i < cur_cloud_ptr_->size(); ++i) {
      base::RadarPointF pt = cur_cloud_ptr_->at(i);
      pcl::PointXYZI new_pt;
      new_pt.x = pt.x;
      new_pt.y = pt.y;
      new_pt.z = pt.z;
      new_pt.intensity = 1;
      cloud.push_back(new_pt);
    }
    if (cloud.points.size() != 0) {
      pcl::io::savePCDFileBinaryCompressed(
      "/apollo/data/pcd/"+std::to_string(frame->timestamp)+".pcd", cloud);
    }
  }

  // after fusing
  while (static_cast<int>(prev_world_clouds_.size()) >=
          param_.preprocess().num_fuse_frames() - 1) {
    prev_world_clouds_.pop_front();
  }
  prev_world_clouds_.emplace_back(cur_world_cloud_ptr);
  AINFO << "num points after fusing: " << num_points;

  // inference
  std::vector<float> out_detections;
  std::vector<int> out_labels;
  point_pillars_ptr_->DoInference(points_array, num_points, &out_detections,
                                  &out_labels);
  inference_time_ = timer.toc(true);

  // transfer output bounding boxes to objects
  GetObjects(&frame->segmented_objects, frame->radar2world_pose,
             out_detections, &out_labels);
  // calculate velocity of object
  // CalObjectVelocity(&frame->segmented_objects, options);
  collect_time_ = timer.toc(true);

  AINFO << "PointPillars: "
        << "\n"
        << "cloud_to_array: " << cloud_to_array_time_ << "\t"
        << "inference: " << inference_time_ << "\t"
        << "collect: " << collect_time_;

  delete[] points_array;
  return true;
}

void Radar4dDetection::FuseCloud(
    const base::RadarPointFCloudPtr& out_cloud_ptr,
    const std::deque<base::RadarPointDCloudPtr>& fuse_clouds,
    float* out_points_array,
    const float normalizing_factor) {
  int j = 0;
  // 当前帧
  for (size_t i = 0; i < out_cloud_ptr->size(); ++i) {
      const auto& point = out_cloud_ptr->at(i);
      if (point.z < z_min_ || point.z > z_max_ ||
          point.y < y_min_ || point.y > y_max_ ||
          point.x < x_min_ || point.x > x_max_) {
        continue;
      }

      // network input point features
      out_points_array[j * param_.preprocess().num_point_feature() + 0] =
        point.x;
      out_points_array[j * param_.preprocess().num_point_feature() + 1] =
        point.y;
      out_points_array[j * param_.preprocess().num_point_feature() + 2] =
        point.z;
      out_points_array[j * param_.preprocess().num_point_feature() + 3] =
        point.rcs;
      out_points_array[j * param_.preprocess().num_point_feature() + 4] =
        point.velocity;
      out_points_array[j * param_.preprocess().num_point_feature() + 5] =
        point.comp_vel;
      out_points_array[j * param_.preprocess().num_point_feature() + 6] =
        0;
      j++;
  }

  // 之前帧
  int frame_number = 0;
  for (auto iter = fuse_clouds.rbegin(); iter != fuse_clouds.rend(); ++iter) {
    frame_number--;
    for (size_t i = 0; i < (*iter)->size(); ++i) {
      // fuse cloud
      auto& point = (*iter)->at(i);
      Eigen::Vector3d trans_point(point.x, point.y, point.z);
      trans_point = radar_frame_ref_->radar2world_pose.inverse() * trans_point;
      base::RadarPointF pt;
      pt.x = static_cast<float>(trans_point(0));
      pt.y = static_cast<float>(trans_point(1));
      pt.z = static_cast<float>(trans_point(2));
      pt.rcs = static_cast<float>(point.rcs);
      pt.velocity = static_cast<float>(point.velocity);
      pt.comp_vel = static_cast<float>(point.comp_vel);
      out_cloud_ptr->push_back(pt);

      if (pt.z < z_min_ || pt.z > z_max_ ||
          pt.y < y_min_ || pt.y > y_max_ ||
          pt.x < x_min_ || pt.x > x_max_) {
        continue;
      }

      // network input point features
      out_points_array[j * param_.preprocess().num_point_feature() + 0] =
        pt.x;
      out_points_array[j * param_.preprocess().num_point_feature() + 1] =
        pt.y;
      out_points_array[j * param_.preprocess().num_point_feature() + 2] =
        pt.z;
      out_points_array[j * param_.preprocess().num_point_feature() + 3] =
        pt.rcs;
      out_points_array[j * param_.preprocess().num_point_feature() + 4] =
        pt.velocity;
      out_points_array[j * param_.preprocess().num_point_feature() + 5] =
        pt.comp_vel;
      out_points_array[j * param_.preprocess().num_point_feature() + 6] =
        frame_number;
      j++;
    }
  }
}

void Radar4dDetection::GetObjects(
    std::vector<std::shared_ptr<Object>>* objects, const Eigen::Affine3d& pose,
    std::vector<float>& detections, std::vector<int>* labels) {
  int num_objects = detections.size() /
    param_.postprocess().num_output_box_feature();

  objects->clear();
  base::ObjectPool::Instance().BatchGet(num_objects, objects);

  for (int i = 0; i < num_objects; ++i) {
    auto& object = objects->at(i);
    object->id = i;

    // read params of bounding box
    float x = detections.at(
      i * param_.postprocess().num_output_box_feature() + 0);
    float y = detections.at(
      i * param_.postprocess().num_output_box_feature() + 1);
    float z = detections.at(
      i * param_.postprocess().num_output_box_feature() + 2);
    float dx = detections.at(
      i * param_.postprocess().num_output_box_feature() + 4);
    float dy = detections.at(
      i * param_.postprocess().num_output_box_feature() + 3);
    float dz = detections.at(
      i * param_.postprocess().num_output_box_feature() + 5);
    float yaw = detections.at(
      i * param_.postprocess().num_output_box_feature() + 6);
    yaw += M_PI / 2;
    yaw = std::atan2(sinf(yaw), cosf(yaw));
    yaw = -yaw;

    // directions
    object->theta = yaw;
    object->direction[0] = cosf(yaw);
    object->direction[1] = sinf(yaw);
    object->direction[2] = 0;
    object->radar4d_supplement.is_orientation_ready = true;

    // compute vertexes of bounding box and transform to world coordinate
    object->radar4d_supplement.num_points_in_roi = 8;
    object->radar4d_supplement.on_use = true;
    object->radar4d_supplement.is_background = false;
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
          RadarPointF point;
          point.x = v3f.x();
          point.y = v3f.y();
          point.z = v3f.z();
          object->radar4d_supplement.cloud.push_back(point);

          Eigen::Vector3d trans_point(point.x, point.y, point.z);
          trans_point = pose * trans_point;
          RadarPointD world_point;
          world_point.x = trans_point(0);
          world_point.y = trans_point(1);
          world_point.z = trans_point(2);
          object->radar4d_supplement.cloud_world.push_back(world_point);
        }
      }
    }

    // classification
    object->radar4d_supplement.raw_probs.push_back(std::vector<float>(
        static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE), 0.f));
    object->radar4d_supplement.raw_classification_methods.push_back(Name());
    object->sub_type = GetObjectSubType(labels->at(i));
    object->type = base::kSubType2TypeMap.at(object->sub_type);
    object->radar4d_supplement.raw_probs.back()[static_cast<int>(
      object->type)] = 1.0f;
    // copy to type
    object->type_probs.assign(
      object->radar4d_supplement.raw_probs.back().begin(),
      object->radar4d_supplement.raw_probs.back().end());
  }

  // std::vector<float> box_corner(num_objects * 8);
  // std::vector<float> box_rectangular(num_objects * 4);
  // GetBoxCorner(num_objects, detections,
  //   box_corner, box_rectangular);
  // GetBoxIndices(num_objects, detections,
  //   box_corner, box_rectangular, objects);
}

base::ObjectSubType Radar4dDetection::GetObjectSubType(const int label) {
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

void Radar4dDetection::GetBoxCorner(int num_objects,
                                    const std::vector<float> &detections,
                                    std::vector<float> &box_corner,
                                    std::vector<float> &box_rectangular) {
  for (int i = 0; i < num_objects; ++i) {
    float x = detections[
      i * param_.postprocess().num_output_box_feature()];
    float y = detections[
      i * param_.postprocess().num_output_box_feature() + 1];
    float w = detections[
      i * param_.postprocess().num_output_box_feature() + 3];
    float l = detections[
      i * param_.postprocess().num_output_box_feature() + 4];
    float a = detections[
      i * param_.postprocess().num_output_box_feature() + 6];

    float cos_a = cos(a);
    float sin_a = sin(a);
    float hw = w * 0.5;
    float hl = l * 0.5;

    float left_up_x = (-hw) * cos_a + (-hl) * sin_a + x;
    float left_up_y = (-hw) * (-sin_a) + (-hl) * cos_a + y;
    float right_up_x = (-hw) * cos_a + (hl) * sin_a + x;
    float right_up_y = (-hw) * (-sin_a) + (hl) * cos_a + y;
    float right_down_x = (hw) * cos_a + (hl) * sin_a + x;
    float right_down_y = (hw) * (-sin_a) + (hl) * cos_a + y;
    float left_down_x = (hw) * cos_a + (-hl) * sin_a + x;
    float left_down_y = (hw) * (-sin_a) + (-hl) * cos_a + y;

    box_corner[i * 8 + 0] = left_up_x;
    box_corner[i * 8 + 1] = left_up_y;
    box_corner[i * 8 + 2] = right_up_x;
    box_corner[i * 8 + 3] = right_up_y;
    box_corner[i * 8 + 4] = right_down_x;
    box_corner[i * 8 + 5] = right_down_y;
    box_corner[i * 8 + 6] = left_down_x;
    box_corner[i * 8 + 7] = left_down_y;

    box_rectangular[i * 4] = std::min(std::min(std::min(left_up_x,
                              right_up_x), right_down_x), left_down_x);
    box_rectangular[i * 4 + 1] = std::min(std::min(std::min(left_up_y,
                                  right_up_y), right_down_y), left_down_y);
    box_rectangular[i * 4 + 2] = std::max(std::max(std::max(left_up_x,
                                  right_up_x), right_down_x), left_down_x);
    box_rectangular[i * 4 + 3] = std::max(std::max(std::max(left_up_y,
                                  right_up_y), right_down_y), left_down_y);
  }
}

void Radar4dDetection::GetBoxIndices(
      int num_objects,
      const std::vector<float> &detections,
      const std::vector<float> &box_corner,
      const std::vector<float> &box_rectangular,
      std::vector<std::shared_ptr<Object>> *objects) {
  for (size_t point_idx = 0; point_idx < original_cloud_->size(); ++point_idx) {
    const auto &point = original_cloud_->at(point_idx);
    float px = point.x;
    float py = point.y;
    float pz = point.z;

    for (int box_idx = 0; box_idx < num_objects; box_idx++) {
      if (px < box_rectangular[box_idx * 4 + 0] ||
          px > box_rectangular[box_idx * 4 + 2]) {
        continue;
      }
      if (py < box_rectangular[box_idx * 4 + 1] ||
          py > box_rectangular[box_idx * 4 + 3]) {
        continue;
      }

      float z = detections[
        box_idx * param_.postprocess().num_output_box_feature() + 2];
      float h = detections[
        box_idx * param_.postprocess().num_output_box_feature() + 5];
      if (pz < (z - h / 2) ||
          pz > (z + h / 2)) {
        continue;
      }

      float x1 = box_corner[box_idx * 8 + 0];
      float x2 = box_corner[box_idx * 8 + 2];
      float x3 = box_corner[box_idx * 8 + 4];
      float x4 = box_corner[box_idx * 8 + 6];
      float y1 = box_corner[box_idx * 8 + 1];
      float y2 = box_corner[box_idx * 8 + 3];
      float y3 = box_corner[box_idx * 8 + 5];
      float y4 = box_corner[box_idx * 8 + 7];

      // double angl1 = (px - x1) * (x2 - x1) + (py - y1) * (y2 - y1);
      // double angl2 = (px - x2) * (x3 - x2) + (py - y2) * (y3 - y2);
      // double angl3 = (px - x3) * (x4 - x3) + (py - y3) * (y4 - y3);
      // double angl4 = (px - x4) * (x1 - x4) + (py - y4) * (y1 - y4);

      double angl1 = (px - x1) * (y2 - y1) - (py - y1) * (x2 - x1);
      double angl2 = (px - x2) * (y3 - y2) - (py - y2) * (x3 - x2);
      double angl3 = (px - x3) * (y4 - y3) - (py - y3) * (x4 - x3);
      double angl4 = (px - x4) * (y1 - y4) - (py - y4) * (x1 - x4);

      if ((angl1 <= 0 && angl2 <= 0 && angl3 <= 0 && angl4 <= 0) ||
          (angl1 >= 0 && angl2 >= 0 && angl3 >= 0 && angl4 >= 0)) {
          auto &object = objects->at(box_idx);
          const auto &world_point = original_world_cloud_->at(point_idx);
          object->radar4d_supplement.point_ids.push_back(point_idx);
          object->radar4d_supplement.cloud.push_back(point);
          object->radar4d_supplement.cloud_world.push_back(world_point);
      }
    }
  }
  for (int i = 0; i < num_objects; i++) {
    auto &object = objects->at(i);
    object->radar4d_supplement.num_points_in_roi =
      object->radar4d_supplement.point_ids.size();
  }
}

// Radar can only measure the doppler velocity, without the tangential velocity
// So, the calculated velocity is not accuracy, and not being used.
void Radar4dDetection::CalObjectVelocity(
  const std::vector<std::shared_ptr<Object>> *objects,
  const DetectorOptions& options) {
  for (uint32_t i=0; i < objects->size(); ++i) {
    auto &object = objects->at(i);

    double obj_vx = 0;
    double obj_vy = 0;
    double obj_vz = 0;
    int count = 0;

    for (auto &point : object->radar4d_supplement.cloud) {
      float azimuth = std::atan2(point.y, point.x);
      float xy_temp = std::sqrt(point.x * point.x
                              + point.y * point.y);
      float elevation = std::atan2(point.z, xy_temp);

      float comp_vel_x = std::cos(elevation) * \
        std::cos(azimuth) * point.comp_vel;
      float comp_vel_y = std::cos(elevation) * \
        std::sin(azimuth) * point.comp_vel;
      float comp_vel_z = std::sin(elevation) * \
        point.comp_vel;
      obj_vx += comp_vel_x;
      obj_vy += comp_vel_y;
      obj_vz += comp_vel_z;
      count++;
    }
    if (count != 0) {
      Eigen::Vector3d vel_average(obj_vx / count,
                                  obj_vy / count,
                                  obj_vz / count);
      object->velocity = vel_average.cast<float>();
    }
  }
}

PERCEPTION_REGISTER_DETECTOR(Radar4dDetection);

}  // namespace radar4d
}  // namespace perception
}  // namespace apollo
