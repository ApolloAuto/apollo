/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <map>
#include <string>
#include <vector>

#include "modules/perception/camera/common/camera_frame.h"
#include "modules/perception/camera/common/camera_ground_plane.h"
#include "modules/perception/camera/common/object_template_manager.h"
#include "modules/perception/camera/lib/obstacle/tracker/omt/frame_list.h"
#include "modules/perception/camera/lib/obstacle/tracker/omt/omt.pb.h"
#include "modules/perception/camera/lib/obstacle/tracker/omt/target.h"

namespace apollo {
namespace perception {
namespace camera {

struct Reference {
  float area = 0.0f;
  float k = 0.0f;
  float ymax = 0.0f;
};

class ObstacleReference {
 public:
  void Init(const omt::ReferenceParam &ref_param, float width, float height);
  void UpdateReference(const CameraFrame *frame,
                       const std::vector<Target> &targets);
  void CorrectSize(CameraFrame *frame);

 private:
  void SyncGroundEstimator(const std::string &sensor,
                           const Eigen::Matrix3f &camera_k_matrix,
                           int img_width, int img_height) {
    if (ground_estimator_mapper_.find(sensor) ==
        ground_estimator_mapper_.end()) {
      auto &ground_estimator = ground_estimator_mapper_[sensor];
      const float fx = camera_k_matrix(0, 0);
      const float fy = camera_k_matrix(1, 1);
      const float cx = camera_k_matrix(0, 2);
      const float cy = camera_k_matrix(1, 2);
      std::vector<float> k_mat = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
      ground_estimator.Init(k_mat, img_width, img_height, common::IRec(fx));
    }
  }

 private:
  omt::ReferenceParam ref_param_;
  std::map<std::string, std::vector<Reference>> reference_;
  std::map<std::string, std::vector<std::vector<int>>> ref_map_;
  std::vector<std::vector<int>> init_ref_map_;
  float img_width_;
  float img_height_;
  int ref_width_;
  int ref_height_;

  // ground estimator W.R.T different cameras
  std::map<std::string, CameraGroundPlaneDetector> ground_estimator_mapper_;

 protected:
  ObjectTemplateManager *object_template_manager_ = nullptr;
};
}  // namespace camera
}  // namespace perception
}  // namespace apollo
