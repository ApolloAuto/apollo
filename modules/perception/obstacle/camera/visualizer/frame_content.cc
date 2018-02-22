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
#include <Eigen/LU>
#include <map>
#include "modules/common/log.h"

namespace apollo {
namespace perception {

FrameContent::FrameContent() : _global_offset_initialized(false) {
  _continuous_type = PC_CONTINUOUS;
}

FrameContent::~FrameContent() {}

void FrameContent::set_image_content(double timestamp, cv::Mat image) {
  ImageContent image_content;
  image_content._timestamp = timestamp;

  image_content._image_mat_src = image;
  _image_caches[timestamp] = image_content;
}

void FrameContent::set_motion_content(double timestamp,
                                      MotionBufferPtr motion_buffer) {
  MotionContent motion_content;
  motion_content._motion_frame_content = *motion_buffer;
  _motion_caches[timestamp] = motion_content;
}

void FrameContent::set_camera_content(double timestamp,
                                      Eigen::Matrix4d pose_c2w,
                                      const std::vector<ObjectPtr>& objects,
                                      const CameraFrameSupplement& supplement) {
  if (_camera_caches.find(timestamp) != _camera_caches.end()) return;

  CameraContent content;
  content._timestamp = timestamp;
  content._pose_c2w = pose_c2w;
  content._camera_objects.resize(objects.size());
  for (size_t i = 0; i < objects.size(); ++i) {
    content._camera_objects[i].reset(new Object);
    content._camera_objects[i]->clone(*objects[i]);
    // offset_object(content._camera_objects[i], _global_offset);
  }
  content._camera_frame_supplement->clone(supplement);
  _camera_caches[timestamp] = content;
}

void FrameContent::set_camera_content(double timestamp,
                                      Eigen::Matrix4d pose_c2w,
                                      const std::vector<ObjectPtr>& objects) {
  if (_camera_caches.find(timestamp) != _camera_caches.end()) return;

  CameraContent content;
  content._timestamp = timestamp;

  if (!_global_offset_initialized) {
    _global_offset[0] = -pose_c2w(0, 3);
    _global_offset[1] = -pose_c2w(1, 3);
    _global_offset[2] = -pose_c2w(2, 3);
    _global_offset_initialized = true;
    AINFO << "initial pose " << pose_c2w;
    AINFO << "offset = " << _global_offset[0] << "  " << _global_offset[1]
          << "  " << _global_offset[2] << "\n";
  }
  content._pose_c2w = pose_c2w;
  // content._pose_c2w(0, 3) += _global_offset[0];
  // content._pose_c2w(1, 3) += _global_offset[1];
  // content._pose_c2w(2, 3) += _global_offset[2];
  content._camera_objects.resize(objects.size());
  for (size_t i = 0; i < objects.size(); ++i) {
    content._camera_objects[i].reset(new Object);
    content._camera_objects[i]->clone(*objects[i]);
    offset_object(content._camera_objects[i], _global_offset);
  }
  _camera_caches[timestamp] = content;
}

void FrameContent::set_radar_content(double timestamp,
                                     const std::vector<ObjectPtr>& objects) {
  RadarContent content;
  content._timestamp = timestamp;

  content._radar_objects.resize(objects.size());
  for (size_t i = 0; i < objects.size(); ++i) {
    content._radar_objects[i].reset(new Object);
    content._radar_objects[i]->clone(*objects[i]);
    offset_object(content._radar_objects[i], _global_offset);
  }
  _radar_caches[timestamp] = content;
}

void FrameContent::set_fusion_content(double timestamp,
                                      const std::vector<ObjectPtr>& objects) {
  FusionContent content;
  content._timestamp = timestamp;

  content._fused_objects.resize(objects.size());
  for (size_t i = 0; i < objects.size(); ++i) {
    content._fused_objects[i].reset(new Object);
    content._fused_objects[i]->clone(*objects[i]);
    offset_object(content._fused_objects[i], _global_offset);
  }
  _fusion_caches[timestamp] = content;
}

void FrameContent::set_gt_content(double timestamp,
                                  const std::vector<ObjectPtr>& objects) {
  GroundTruthContent content;
  content._timestamp = timestamp;

  content._gt_objects.resize(objects.size());
  for (size_t i = 0; i < objects.size(); ++i) {
    content._gt_objects[i].reset(new Object);
    content._gt_objects[i]->clone(*objects[i]);
    offset_object(content._gt_objects[i], _global_offset);
  }
  _gt_caches[timestamp] = content;
}

void FrameContent::update_timestamp(double ref) {
  double best_delta = FLT_MAX;
  double best_ts = -1;
  if (_continuous_type == IMAGE_CONTINUOUS) {
    if (_image_caches.empty()) {
      return;
    }
    best_ts = FLT_MAX;
    for (std::map<double, ImageContent>::iterator it = _image_caches.begin();
         it != _image_caches.end(); it++) {
      double it_ts = it->first;
      if (it_ts < best_ts && _current_image_timestamp != it_ts) {
        best_ts = it_ts;
      }
    }
    ref = best_ts;
    _current_image_timestamp = ref;
  }
  for (std::map<double, ImageContent>::iterator it = _image_caches.begin();
       it != _image_caches.end();) {
    if (it->first < _current_image_timestamp) {
      _image_caches.erase(it++);
    } else {
      it++;
    }
  }

  std::string ts_string = std::to_string(ref);
  AINFO << "cur time: " << ts_string
        << " | radar caches num: " << _radar_caches.size()
        << " | camera caches num: " << _camera_caches.size()
        << " | fusion caches num: " << _fusion_caches.size()
        << " | image caches num: " << _image_caches.size();
  best_delta = FLT_MAX;
  best_ts = -1;

  for (std::map<double, RadarContent>::iterator it = _radar_caches.begin();
       it != _radar_caches.end(); it++) {
    double it_ts = it->first;
    double delta = fabs(it_ts - ref);

    if (delta < best_delta) {
      best_delta = delta;
      best_ts = it_ts;
    }
  }
  _current_radar_timestamp = best_ts;
  for (std::map<double, RadarContent>::iterator it = _radar_caches.begin();
       it != _radar_caches.end();) {
    if (it->first < best_ts) {
      _radar_caches.erase(it++);
    } else {
      it++;
    }
  }
  best_delta = FLT_MAX;
  best_ts = -1;
  for (std::map<double, FusionContent>::iterator it = _fusion_caches.begin();
       it != _fusion_caches.end(); it++) {
    double it_ts = it->first;
    double delta = fabs(it_ts - ref);

    if (delta < best_delta) {
      best_delta = delta;
      best_ts = it_ts;
    }
  }
  _current_fusion_timestamp = best_ts;
  for (std::map<double, FusionContent>::iterator it = _fusion_caches.begin();
       it != _fusion_caches.end();) {
    if (it->first < best_ts) {
      _fusion_caches.erase(it++);
    } else {
      it++;
    }
  }

  // find camera tracked best ts
  best_delta = FLT_MAX;
  best_ts = -1;
  for (std::map<double, CameraContent>::iterator it = _camera_caches.begin();
       it != _camera_caches.end(); it++) {
    double it_ts = it->first;
    double delta = fabs(it_ts - ref);

    if (delta < best_delta) {
      best_delta = delta;
      best_ts = it_ts;
    }
  }
  _current_camera_timestamp = best_ts;
  for (std::map<double, CameraContent>::iterator it = _camera_caches.begin();
       it != _camera_caches.end();) {
    if (it->first < best_ts) {
      _camera_caches.erase(it++);
    } else {
      it++;
    }
  }

  best_delta = FLT_MAX;
  best_ts = -1;
  for (std::map<double, GroundTruthContent>::iterator it = _gt_caches.begin();
       it != _gt_caches.end(); it++) {
    double it_ts = it->first;
    double delta = fabs(it_ts - ref);
    if (delta < best_delta) {
      best_delta = delta;
      best_ts = it_ts;
    }
  }
  _current_gt_timestamp = best_ts;
  for (std::map<double, GroundTruthContent>::iterator it = _gt_caches.begin();
       it != _gt_caches.end();) {
    if (it->first < best_ts) {
      _gt_caches.erase(it++);
    } else {
      it++;
    }
  }

  // get motion timestamp
  best_delta = FLT_MAX;
  best_ts = -1;
  for (std::map<double, MotionContent>::iterator it = _motion_caches.begin();
       it != _motion_caches.end(); it++) {
    double it_ts = it->first;
    double delta = fabs(it_ts - ref);

    if (delta < best_delta) {
      best_delta = delta;
      best_ts = it_ts;
    }
  }
  _current_motion_timestamp = best_ts;
  for (std::map<double, MotionContent>::iterator it = _motion_caches.begin();
       it != _motion_caches.end();) {
    if (it->first < best_ts) {
      _motion_caches.erase(it++);
    } else {
      it++;
    }
  }
}

Eigen::Matrix4d FrameContent::get_camera_to_world_pose() {
  std::map<double, CameraContent>::iterator it =
      _camera_caches.find(_current_camera_timestamp);
  if (it == _camera_caches.end()) {
    return Eigen::Matrix4d::Identity();
  }
  CameraContent content = it->second;
  return content._pose_c2w;
}

cv::Mat FrameContent::get_camera_image() {
  std::map<double, ImageContent>::iterator it =
      _image_caches.find(_current_image_timestamp);
  if (it == _image_caches.end()) {
    cv::Mat mat = cv::Mat::zeros(1080, 1920, CV_8UC3);
    return mat;
  }
  ImageContent content = it->second;
  return content._image_mat_src;
}

const MotionBuffer FrameContent::get_motion_buffer() {
  std::map<double, MotionContent>::iterator it =
      _motion_caches.find(_current_motion_timestamp);
  if (it == _motion_caches.end()) {
    return MotionBuffer(0);
  }
  MotionContent content = it->second;
  return content._motion_frame_content;
}

void FrameContent::set_camera2car_pose(Eigen::Matrix4d pose_velo2cam) {
  _pose_camera2velo = pose_velo2cam.inverse();
}

Eigen::Matrix4d FrameContent::get_opengl_camera_system_pose() {
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();

  if (_continuous_type == IMAGE_CONTINUOUS) {
    pose = get_camera_to_world_pose() * _pose_camera2velo;
  }
  return pose;
}

std::vector<ObjectPtr> FrameContent::get_radar_objects() {
  std::map<double, RadarContent>::iterator it =
      _radar_caches.find(_current_radar_timestamp);
  if (it == _radar_caches.end()) {
    return std::vector<ObjectPtr>();
  }
  RadarContent content = it->second;
  return content._radar_objects;
}

std::vector<ObjectPtr> FrameContent::get_camera_objects() {
  std::map<double, CameraContent>::iterator it =
      _camera_caches.find(_current_camera_timestamp);
  if (it == _camera_caches.end()) {
    return std::vector<ObjectPtr>();
  }
  CameraContent content = it->second;
  return content._camera_objects;
}

CameraFrameSupplementPtr FrameContent::get_camera_frame_supplement() {
  std::map<double, CameraContent>::iterator it =
      _camera_caches.find(_current_camera_timestamp);
  if (it == _camera_caches.end()) {
    CameraFrameSupplementPtr supplement_ptr;
    supplement_ptr.reset(new CameraFrameSupplement);
    return supplement_ptr;
  }
  CameraContent content = it->second;
  return content._camera_frame_supplement;
}

double FrameContent::get_visualization_timestamp() {
  double timestamp = 0;
  if (_continuous_type == IMAGE_CONTINUOUS) {
    timestamp = _current_image_timestamp;
  }
  return timestamp;
}

std::vector<ObjectPtr> FrameContent::get_fused_objects() {
  std::map<double, FusionContent>::iterator it =
      _fusion_caches.find(_current_fusion_timestamp);
  if (it == _fusion_caches.end()) {
    return std::vector<ObjectPtr>();
  }
  FusionContent content = it->second;
  return content._fused_objects;
}

std::vector<ObjectPtr> FrameContent::get_gt_objects() {
  std::map<double, GroundTruthContent>::iterator it =
      _gt_caches.find(_current_gt_timestamp);
  if (it == _gt_caches.end()) {
    return std::vector<ObjectPtr>();
  }
  GroundTruthContent content = it->second;
  return content._gt_objects;
}

void FrameContent::offset_object(ObjectPtr object,
                                 const Eigen::Vector3d& offset) {
  object->center[0] += offset[0];
  object->center[1] += offset[1];
  object->center[2] += offset[2];
}

}  // namespace perception
}  // namespace apollo
