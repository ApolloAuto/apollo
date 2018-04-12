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
#include <memory>
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
namespace lowcostvisualizer {

class BaseContent {
 public:
  BaseContent() = default;

  ~BaseContent() = default;
  double timestamp_ = -1.0;
};

class CameraContent : public BaseContent {
 public:
  CameraContent()
      : camera_frame_supplement_(new CameraFrameSupplement),
        _pose_c2w(Eigen::Matrix4d::Identity()) {}
  std::vector<std::shared_ptr<Object>> camera_objects_;
  CameraFrameSupplementPtr camera_frame_supplement_;
  Eigen::Matrix4d _pose_c2w;
};

class ImageContent : public BaseContent {
 public:
  cv::Mat image_mat_src_;
};

class MotionContent : public BaseContent {
 public:
  //    MotionContent() : motion_frame_content_(new MotionBuffer) {}
  //    MotionBufferPtr motion_frame_content_;
  //    MotionContent() {}
  MotionBuffer motion_frame_content_;
};

class RadarContent : public BaseContent {
 public:
  RadarContent()
      : _pose_fr2w(Eigen::Matrix4d::Identity()),
        _pose_br2w(Eigen::Matrix4d::Identity()) {}
  Eigen::Matrix4d _pose_fr2w;
  // RadarRawObstacles radar_raw_front_;
  Eigen::Matrix4d _pose_br2w;
  // RadarRawObstacles radar_raw_back_;
  std::vector<std::shared_ptr<Object>> radar_objects_;
};

class FusionContent : public BaseContent {
 public:
  std::vector<std::shared_ptr<Object>> fused_objects_;
};

class LaneContent : public BaseContent {
 public:
  LaneObjects lane_objects_;
};

class GroundTruthContent : public BaseContent {
 public:
  std::vector<std::shared_ptr<Object>> gt_objects_;
};

class ContentComparison {
  bool reverse;

 public:
  explicit ContentComparison(const bool& revparam = false) {
    reverse = revparam;
  }

  bool operator()(const BaseContent& lhs, const BaseContent& rhs) const {
    if (reverse) {
      return (lhs.timestamp_ > rhs.timestamp_);
    } else {
      return (lhs.timestamp_ < rhs.timestamp_);
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
                          const std::vector<std::shared_ptr<Object>>& objects,
                          const CameraFrameSupplement& supplement);
  void set_camera_content(double timestamp, Eigen::Matrix4d pose_c2w,
                          const std::vector<std::shared_ptr<Object>>& objects);

  void set_radar_content(double timestamp,
                         const std::vector<std::shared_ptr<Object>>& objects);
  void set_fusion_content(double timestamp,
                          const std::vector<std::shared_ptr<Object>>& objects);
  void set_lane_content(double timestamp,
                        const apollo::perception::LaneObjects& objects);

  void set_gt_content(double timestamp,
                      const std::vector<std::shared_ptr<Object>>& objects);
  void set_camera2car_pose(Eigen::Matrix4d pose_cam2velo);

  void set_motion_content(double timestamp, MotionBufferPtr motion_buffer);
  Eigen::Matrix4d get_opengl_camera_system_pose();
  Eigen::Matrix4d get_camera_to_world_pose();
  Eigen::Matrix4d get_pose_v2w();
  cv::Mat get_camera_image();

  int get_pose_type() { return continuous_type_; }

  void set_pose_type(int type) { continuous_type_ = type; }

  std::vector<std::shared_ptr<Object>> get_camera_objects();
  std::vector<std::shared_ptr<Object>> get_radar_objects();
  double get_visualization_timestamp();

  inline bool has_radar_data() { return radar_caches_.size(); }

  CameraFrameSupplementPtr get_camera_frame_supplement();

  inline bool has_camera_data() { return camera_caches_.size(); }
  /*   inline void set_camera2velo_pose(const Eigen::Matrix4d& pose) {
         _pose_camera2velo = pose;
     }*/

  // fused output
  std::vector<std::shared_ptr<Object>> get_fused_objects();
  // gt
  std::vector<std::shared_ptr<Object>> get_gt_objects();
  // lane objects
  apollo::perception::LaneObjects get_lane_objects();

  const MotionBuffer get_motion_buffer();

 protected:
  // coordinate transform utilities
  void offset_object(std::shared_ptr<Object> object,
                     const Eigen::Vector3d& offset);

 private:
  const double kEpsilon_ = 1e-6;

  int64_t DoubleToMapKey(const double d) {
    return static_cast<int64_t>(d / kEpsilon_);
  }

  double MapKeyToDouble(const int64_t key) { return key * kEpsilon_; }

  // input
  // 1.radar
  std::map<int64_t, RadarContent> radar_caches_;
  double current_radar_timestamp_;

  // 2.camera
  std::map<int64_t, CameraContent> camera_caches_;
  double current_camera_timestamp_;

  // 3.fusion
  std::map<int64_t, FusionContent> fusion_caches_;
  double current_fusion_timestamp_;

  // 4.ground truth
  std::map<int64_t, GroundTruthContent> gt_caches_;
  double current_gt_timestamp_;

  // 5.image
  std::map<int64_t, ImageContent> image_caches_;
  double current_image_timestamp_;

  // 6.motion
  std::map<int64_t, MotionContent> motion_caches_;
  double current_motion_timestamp_;

  // 7.lane lines
  std::map<int64_t, LaneContent> lane_caches_;
  double current_lane_timestamp_;

  Eigen::Vector3d global_offset_;
  bool global_offset_initialized_;
  int continuous_type_;
  Eigen::Matrix4d _pose_camera2velo;

  DISALLOW_COPY_AND_ASSIGN(FrameContent);
};

}  // namespace lowcostvisualizer
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_VISUALIZER_FRAME_CONTENT_H_
