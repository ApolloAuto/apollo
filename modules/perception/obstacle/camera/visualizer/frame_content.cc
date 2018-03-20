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

#include "modules/perception/obstacle/camera/visualizer/frame_content.h"

#include "Eigen/LU"

#include "modules/common/log.h"
#include "modules/common/util/util.h"

namespace apollo {
namespace perception {
namespace lowcostvisualizer {

FrameContent::FrameContent() : global_offset_initialized_(false) {
  continuous_type_ = PC_CONTINUOUS;
}

FrameContent::~FrameContent() {}

void FrameContent::set_image_content(double timestamp, cv::Mat image) {
  ImageContent image_content;
  image_content.timestamp_ = timestamp;

  image_content.image_mat_src_ = image;
  image_caches_[DoubleToMapKey(timestamp)] = image_content;
}

void FrameContent::set_motion_content(double timestamp,
                                      MotionBufferPtr motion_buffer) {
  MotionContent motion_content;
  motion_content.motion_frame_content_ = *motion_buffer;
  motion_caches_[DoubleToMapKey(timestamp)] = motion_content;
}

void FrameContent::set_camera_content(double timestamp,
                                      Eigen::Matrix4d pose_c2w,
                                      const std::vector<ObjectPtr>& objects,
                                      const CameraFrameSupplement& supplement) {
  if (camera_caches_.find(DoubleToMapKey(timestamp)) != camera_caches_.end()) {
    return;
  }

  CameraContent content;
  content.timestamp_ = timestamp;
  content._pose_c2w = pose_c2w;
  content.camera_objects_.resize(objects.size());
  for (size_t i = 0; i < objects.size(); ++i) {
    content.camera_objects_[i].reset(new Object);
    content.camera_objects_[i]->clone(*objects[i]);
    // offset_object(content.camera_objects_[i], global_offset_);
  }
  content.camera_frame_supplement_->clone(supplement);
  camera_caches_[DoubleToMapKey(timestamp)] = content;
}

void FrameContent::set_camera_content(double timestamp,
                                      Eigen::Matrix4d pose_c2w,
                                      const std::vector<ObjectPtr>& objects) {
  if (camera_caches_.find(DoubleToMapKey(timestamp)) != camera_caches_.end()) {
    return;
  }

  CameraContent content;
  content.timestamp_ = timestamp;

  if (!global_offset_initialized_) {
    global_offset_[0] = -pose_c2w(0, 3);
    global_offset_[1] = -pose_c2w(1, 3);
    global_offset_[2] = -pose_c2w(2, 3);
    global_offset_initialized_ = true;
    AINFO << "initial pose " << pose_c2w;
    AINFO << "offset = " << global_offset_[0] << "  " << global_offset_[1]
          << "  " << global_offset_[2] << "\n";
  }
  content._pose_c2w = pose_c2w;
  // content._pose_c2w(0, 3) += global_offset_[0];
  // content._pose_c2w(1, 3) += global_offset_[1];
  // content._pose_c2w(2, 3) += global_offset_[2];
  content.camera_objects_.resize(objects.size());
  for (size_t i = 0; i < objects.size(); ++i) {
    content.camera_objects_[i].reset(new Object);
    content.camera_objects_[i]->clone(*objects[i]);
    offset_object(content.camera_objects_[i], global_offset_);
  }
  camera_caches_[DoubleToMapKey(timestamp)] = content;
}

void FrameContent::set_radar_content(double timestamp,
                                     const std::vector<ObjectPtr>& objects) {
  RadarContent content;
  content.timestamp_ = timestamp;

  content.radar_objects_.resize(objects.size());
  for (size_t i = 0; i < objects.size(); ++i) {
    content.radar_objects_[i].reset(new Object);
    content.radar_objects_[i]->clone(*objects[i]);
    offset_object(content.radar_objects_[i], global_offset_);
  }
  radar_caches_[DoubleToMapKey(timestamp)] = content;
}

void FrameContent::set_fusion_content(double timestamp,
                                      const std::vector<ObjectPtr>& objects) {
  FusionContent content;
  content.timestamp_ = timestamp;

  content.fused_objects_.resize(objects.size());
  for (size_t i = 0; i < objects.size(); ++i) {
    content.fused_objects_[i].reset(new Object);
    content.fused_objects_[i]->clone(*objects[i]);
    offset_object(content.fused_objects_[i], global_offset_);
  }
  fusion_caches_[DoubleToMapKey(timestamp)] = content;
}

void FrameContent::set_gt_content(double timestamp,
                                  const std::vector<ObjectPtr>& objects) {
  GroundTruthContent content;
  content.timestamp_ = timestamp;

  content.gt_objects_.resize(objects.size());
  for (size_t i = 0; i < objects.size(); ++i) {
    content.gt_objects_[i].reset(new Object);
    content.gt_objects_[i]->clone(*objects[i]);
    offset_object(content.gt_objects_[i], global_offset_);
  }
  gt_caches_[DoubleToMapKey(timestamp)] = content;
}

void FrameContent::update_timestamp(double ref) {
  double best_delta = FLT_MAX;
  double best_ts = -1;
  if (continuous_type_ == IMAGE_CONTINUOUS) {
    if (image_caches_.empty()) {
      return;
    }
    best_ts = FLT_MAX;
    for (auto it = image_caches_.begin(); it != image_caches_.end(); ++it) {
      double it_ts = it->first;
      if (it_ts < best_ts && current_image_timestamp_ != it_ts) {
        best_ts = it_ts;
      }
    }
    ref = best_ts;
    current_image_timestamp_ = ref;
  }
  common::util::erase_map_where(
      image_caches_, [this](std::map<int64_t, ImageContent>::value_type& p) {
        return this->MapKeyToDouble(p.first) < this->current_image_timestamp_;
      });

  std::string ts_string = std::to_string(ref);
  AINFO << "cur time: " << ts_string
        << " | radar caches num: " << radar_caches_.size()
        << " | camera caches num: " << camera_caches_.size()
        << " | fusion caches num: " << fusion_caches_.size()
        << " | image caches num: " << image_caches_.size();
  best_delta = FLT_MAX;
  best_ts = -1;

  for (std::map<int64_t, RadarContent>::iterator it = radar_caches_.begin();
       it != radar_caches_.end(); ++it) {
    double it_ts = MapKeyToDouble(it->first);
    double delta = fabs(it_ts - ref);

    if (delta < best_delta) {
      best_delta = delta;
      best_ts = it_ts;
    }
  }
  current_radar_timestamp_ = best_ts;
  common::util::erase_map_where(
      radar_caches_,
      [this, best_ts](std::map<int64_t, RadarContent>::value_type& p) {
        return this->MapKeyToDouble(p.first) < best_ts;
      });

  best_delta = FLT_MAX;
  best_ts = -1;
  for (auto it = fusion_caches_.begin(); it != fusion_caches_.end(); ++it) {
    double it_ts = it->first;
    double delta = fabs(it_ts - ref);

    if (delta < best_delta) {
      best_delta = delta;
      best_ts = it_ts;
    }
  }
  current_fusion_timestamp_ = best_ts;
  common::util::erase_map_where(
      fusion_caches_,
      [this, best_ts](std::map<int64_t, FusionContent>::value_type& p) {
        return this->MapKeyToDouble(p.first) < best_ts;
      });

  // find camera tracked best ts
  best_delta = FLT_MAX;
  best_ts = -1;
  for (auto it = camera_caches_.begin(); it != camera_caches_.end(); ++it) {
    double it_ts = MapKeyToDouble(it->first);
    double delta = fabs(it_ts - ref);

    if (delta < best_delta) {
      best_delta = delta;
      best_ts = it_ts;
    }
  }
  current_camera_timestamp_ = best_ts;
  common::util::erase_map_where(
      camera_caches_,
      [this, best_ts](std::map<int64_t, CameraContent>::value_type& p) {
        return this->MapKeyToDouble(p.first) < best_ts;
      });

  best_delta = FLT_MAX;
  best_ts = -1;
  for (auto it = gt_caches_.begin(); it != gt_caches_.end(); ++it) {
    double it_ts = it->first;
    double delta = fabs(it_ts - ref);
    if (delta < best_delta) {
      best_delta = delta;
      best_ts = it_ts;
    }
  }
  current_gt_timestamp_ = best_ts;
  common::util::erase_map_where(
      gt_caches_,
      [this, best_ts](std::map<int64_t, GroundTruthContent>::value_type& p) {
        return this->MapKeyToDouble(p.first) < best_ts;
      });

  // get motion timestamp
  best_delta = FLT_MAX;
  best_ts = -1;
  for (auto it = motion_caches_.begin(); it != motion_caches_.end(); ++it) {
    double it_ts = it->first;
    double delta = fabs(it_ts - ref);

    if (delta < best_delta) {
      best_delta = delta;
      best_ts = it_ts;
    }
  }
  current_motion_timestamp_ = best_ts;
  common::util::erase_map_where(
      motion_caches_,
      [this, best_ts](std::map<int64_t, MotionContent>::value_type& p) {
        return this->MapKeyToDouble(p.first) < best_ts;
      });
}

Eigen::Matrix4d FrameContent::get_camera_to_world_pose() {
  auto it = camera_caches_.find(DoubleToMapKey(current_camera_timestamp_));
  if (it == camera_caches_.end()) {
    return Eigen::Matrix4d::Identity();
  }
  CameraContent content = it->second;
  return content._pose_c2w;
}

cv::Mat FrameContent::get_camera_image() {
  // TODO(later): Hotfix now. Check timestamp key correctness later
  if (!image_caches_.empty()) {
    auto it = image_caches_.rbegin();
    return it->second.image_mat_src_;
  }

  auto it = image_caches_.find(DoubleToMapKey(current_image_timestamp_));
  if (it == image_caches_.end()) {
    AWARN << "FrameContent::get_camera_image() : No image found";
    AWARN << "Key: " << DoubleToMapKey(current_image_timestamp_);
    cv::Mat mat = cv::Mat::zeros(1080, 1920, CV_8UC3);
    return mat;
  }
  ImageContent content = it->second;
  return content.image_mat_src_;
}

const MotionBuffer FrameContent::get_motion_buffer() {
  auto it = motion_caches_.find(DoubleToMapKey(current_motion_timestamp_));
  if (it == motion_caches_.end()) {
    return MotionBuffer(0);
  }
  MotionContent content = it->second;
  return content.motion_frame_content_;
}

void FrameContent::set_camera2car_pose(Eigen::Matrix4d pose_velo2cam) {
  _pose_camera2velo = pose_velo2cam.inverse();
}

Eigen::Matrix4d FrameContent::get_opengl_camera_system_pose() {
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();

  if (continuous_type_ == IMAGE_CONTINUOUS) {
    pose = get_camera_to_world_pose() * _pose_camera2velo;
  }
  return pose;
}

std::vector<ObjectPtr> FrameContent::get_radar_objects() {
  auto it = radar_caches_.find(DoubleToMapKey(current_radar_timestamp_));
  if (it == radar_caches_.end()) {
    return std::vector<ObjectPtr>();
  }
  RadarContent content = it->second;
  return content.radar_objects_;
}

std::vector<ObjectPtr> FrameContent::get_camera_objects() {
  auto it = camera_caches_.find(DoubleToMapKey(current_camera_timestamp_));
  if (it == camera_caches_.end()) {
    return std::vector<ObjectPtr>();
  }
  CameraContent content = it->second;
  return content.camera_objects_;
}

CameraFrameSupplementPtr FrameContent::get_camera_frame_supplement() {
  auto it = camera_caches_.find(DoubleToMapKey(current_camera_timestamp_));
  if (it == camera_caches_.end()) {
    CameraFrameSupplementPtr supplement_ptr;
    supplement_ptr.reset(new CameraFrameSupplement);
    return supplement_ptr;
  }
  CameraContent content = it->second;
  return content.camera_frame_supplement_;
}

double FrameContent::get_visualization_timestamp() {
  double timestamp = 0;
  if (continuous_type_ == IMAGE_CONTINUOUS) {
    timestamp = current_image_timestamp_;
  }
  return timestamp;
}

std::vector<ObjectPtr> FrameContent::get_fused_objects() {
  auto it = fusion_caches_.find(DoubleToMapKey(current_fusion_timestamp_));
  if (it == fusion_caches_.end()) {
    return std::vector<ObjectPtr>();
  }
  FusionContent content = it->second;
  return content.fused_objects_;
}

std::vector<ObjectPtr> FrameContent::get_gt_objects() {
  auto it = gt_caches_.find(DoubleToMapKey(current_gt_timestamp_));
  if (it == gt_caches_.end()) {
    return std::vector<ObjectPtr>();
  }
  GroundTruthContent content = it->second;
  return content.gt_objects_;
}

void FrameContent::offset_object(ObjectPtr object,
                                 const Eigen::Vector3d& offset) {
  object->center[0] += offset[0];
  object->center[1] += offset[1];
  object->center[2] += offset[2];
}

}  // namespace lowcostvisualizer
}  // namespace perception
}  // namespace apollo
