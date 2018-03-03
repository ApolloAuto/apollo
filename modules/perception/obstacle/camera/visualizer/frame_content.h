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

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_VISUALIZER_FRAME_CONTENT_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_VISUALIZER_FRAME_CONTENT_H_

#include <deque>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "boost/shared_ptr.hpp"

#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/base/object_supplement.h"
#include "modules/perception/obstacle/radar/interface/base_radar_detector.h"

namespace apollo {
namespace perception {

class BaseContent {
 public:
  BaseContent() = default;

  ~BaseContent() = default;
  double _timestamp = -1.0;
};

class CameraContent : public BaseContent {
 public:
  CameraContent()
      : _camera_frame_supplement(new CameraFrameSupplement),
        _pose_c2w(Eigen::Matrix4d::Identity()) {}
  std::vector<ObjectPtr> _camera_objects;
  CameraFrameSupplementPtr _camera_frame_supplement;
  Eigen::Matrix4d _pose_c2w;
};

class ImageContent : public BaseContent {
 public:
  cv::Mat _image_mat_src;
};

class MotionContent : public BaseContent {
 public:
  //    MotionContent() : _motion_frame_content(new MotionBuffer) {}
  //    MotionBufferPtr _motion_frame_content;
  //    MotionContent() {}
  MotionBuffer _motion_frame_content;
};

class RadarContent : public BaseContent {
 public:
  RadarContent()
      : _pose_fr2w(Eigen::Matrix4d::Identity()),
        _pose_br2w(Eigen::Matrix4d::Identity()) {}
  Eigen::Matrix4d _pose_fr2w;
  // RadarRawObstacles _radar_raw_front;
  Eigen::Matrix4d _pose_br2w;
  // RadarRawObstacles _radar_raw_back;
  std::vector<ObjectPtr> _radar_objects;
};

class FusionContent : public BaseContent {
 public:
  std::vector<ObjectPtr> _fused_objects;
};

class GroundTruthContent : public BaseContent {
 public:
  std::vector<ObjectPtr> _gt_objects;
};

class ContentComparison {
  bool reverse;

 public:
  explicit ContentComparison(const bool& revparam = false) {
    reverse = revparam;
  }

  bool operator()(const BaseContent& lhs, const BaseContent& rhs) const {
    if (reverse) {
      return (lhs._timestamp > rhs._timestamp);
    } else {
      return (lhs._timestamp < rhs._timestamp);
    }
  }
};

class FrameContent {
 public:
  FrameContent();
  ~FrameContent();
  enum { PC_CONTINUOUS, IMAGE_CONTINUOUS, NONE_CONTINUOUS };
  void update_timestamp(double ref);
  void set_image_content(double timestamp, cv::Mat image);
  void set_camera_content(double timestamp, Eigen::Matrix4d pose_c2w,
                          const std::vector<ObjectPtr>& objects,
                          const CameraFrameSupplement& supplement);
  void set_camera_content(double timestamp, Eigen::Matrix4d pose_c2w,
                          const std::vector<ObjectPtr>& objects);

  void set_radar_content(double timestamp,
                         const std::vector<ObjectPtr>& objects);
  void set_fusion_content(double timestamp,
                          const std::vector<ObjectPtr>& objects);
  void set_gt_content(double timestamp, const std::vector<ObjectPtr>& objects);
  void set_camera2car_pose(Eigen::Matrix4d pose_cam2velo);

  void set_motion_content(double timestamp, MotionBufferPtr motion_buffer);
  Eigen::Matrix4d get_opengl_camera_system_pose();
  Eigen::Matrix4d get_camera_to_world_pose();
  Eigen::Matrix4d get_pose_v2w();
  cv::Mat get_camera_image();

  int get_pose_type() { return _continuous_type; }

  void set_pose_type(int type) { _continuous_type = type; }

  std::vector<ObjectPtr> get_camera_objects();
  std::vector<ObjectPtr> get_radar_objects();
  double get_visualization_timestamp();

  inline bool has_radar_data() { return _radar_caches.size(); }

  CameraFrameSupplementPtr get_camera_frame_supplement();

  inline bool has_camera_data() { return _camera_caches.size(); }
  /*   inline void set_camera2velo_pose(const Eigen::Matrix4d& pose) {
         _pose_camera2velo = pose;
     }*/

  // fused output
  std::vector<ObjectPtr> get_fused_objects();
  // gt
  std::vector<ObjectPtr> get_gt_objects();

  const MotionBuffer get_motion_buffer();

 protected:
  // coordinate transform utilities
  void offset_object(ObjectPtr object, const Eigen::Vector3d& offset);

 private:
  const double kEpsilon_ = 1e-6;

  int64_t DoubleToMapKey(const double d) {
    return static_cast<int64_t>(d / kEpsilon_);
  }

  double MapKeyToDouble(const int64_t key) { return key * kEpsilon_; }

  // input
  // 1.radar
  std::map<int64_t, RadarContent> _radar_caches;
  double _current_radar_timestamp;

  // 2.camera
  std::map<int64_t, CameraContent> _camera_caches;
  double _current_camera_timestamp;

  // 3.fusion
  std::map<int64_t, FusionContent> _fusion_caches;
  double _current_fusion_timestamp;

  // 4.ground truth
  std::map<int64_t, GroundTruthContent> _gt_caches;
  double _current_gt_timestamp;

  // 5.image
  std::map<int64_t, ImageContent> _image_caches;
  double _current_image_timestamp;

  // 6.motion
  std::map<int64_t, MotionContent> _motion_caches;
  double _current_motion_timestamp;

  Eigen::Vector3d _global_offset;
  bool _global_offset_initialized;
  int _continuous_type;
  Eigen::Matrix4d _pose_camera2velo;

  DISALLOW_COPY_AND_ASSIGN(FrameContent);
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_VISUALIZER_FRAME_CONTENT_H_
