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

#include "modules/perception/base/distortion_model.h"
#include "modules/perception/base/point.h"
#include "modules/perception/base/traffic_light.h"
#include "modules/perception/camera/lib/traffic_light/preprocessor/pose.h"

namespace apollo {
namespace perception {
namespace camera {

struct ProjectOption {
  explicit ProjectOption(const std::string& name) : camera_name(name) {}
  std::string camera_name;
};

struct MultiCamerasInitOption {
  std::vector<std::string> camera_names;
};

// @brief Camera Projection project the Light into the image.
class MultiCamerasProjection {
 public:
  MultiCamerasProjection() {}

  ~MultiCamerasProjection() = default;
  bool Init(const MultiCamerasInitOption& options);
  bool Project(const CarPose& pose, const ProjectOption& option,
               base::TrafficLight* light) const;
  bool HasCamera(const std::string& camera_name) const;

  int getImageWidth(const std::string& camera_name) const;
  int getImageHeight(const std::string& camera_name) const;

  const std::vector<std::string>& getCameraNamesByDescendingFocalLen() const {
    return camera_names_;
  }

 private:
  bool BoundaryBasedProject(
      const base::BrownCameraDistortionModelPtr camera_model,
      const Eigen::Matrix4d& c2w_pose,
      const std::vector<base::PointXYZID>& point,
      base::TrafficLight* light) const;

 private:
  // sorted by focal length in descending order
  std::vector<std::string> camera_names_;
  // camera_name -> camera_model
  std::map<std::string, base::BrownCameraDistortionModelPtr> camera_models_;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
