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

#include "modules/perception/common/base/distortion_model.h"
#include "modules/perception/common/base/point.h"
#include "modules/perception/common/base/traffic_light.h"
#include "modules/perception/common/camera/common/pose.h"

namespace apollo {
namespace perception {
namespace trafficlight {

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
  /**
   * @brief Construct a new multi cameras projection object.
   * 
   */
  MultiCamerasProjection() {}
  /**
   * @brief Destroy the multi cameras projection object.
   * 
   */
  ~MultiCamerasProjection() = default;
  /**
   * @brief multicamera project initialization parameters.
   * 
   * @param options 
   * @return true 
   * @return false 
   */
  bool Init(const MultiCamerasInitOption& options);
  /**
   * @brief Boundary projection based on vehicle position and traffic lights.
   * 
   * @param pose 
   * @param option 
   * @param light 
   * @return true 
   * @return false 
   */
  bool Project(const camera::CarPose& pose, const ProjectOption& option,
               base::TrafficLight* light) const;
  /**
   * @brief Determine whether the camera exists.
   * 
   * @param camera_name 
   * @return true 
   * @return false 
   */
  bool HasCamera(const std::string& camera_name) const;
  /**
   * @brief Get the image width object.
   * 
   * @param camera_name 
   * @return int 
   */
  int getImageWidth(const std::string& camera_name) const;
  /**
   * @brief Get the image height object.
   * 
   * @param camera_name 
   * @return int 
   */
  int getImageHeight(const std::string& camera_name) const;
  /**
   * @brief Get the camera names by descending focal len object.
   * 
   * @return const std::vector<std::string>& 
   */
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

}  // namespace trafficlight
}  // namespace perception
}  // namespace apollo
