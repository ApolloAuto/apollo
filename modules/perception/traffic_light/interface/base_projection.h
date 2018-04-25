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
#ifndef MODULES_PERCEPTION_TRAFFIC_LIGHT_INTERFACE_BASE_PROJECTION_H_
#define MODULES_PERCEPTION_TRAFFIC_LIGHT_INTERFACE_BASE_PROJECTION_H_

#include <string>
#include <vector>

#include "Eigen/Core"

#include "modules/perception/lib/base/registerer.h"
#include "modules/perception/lib/config_manager/calibration_config_manager.h"
#include "modules/perception/traffic_light/base/light.h"
#include "modules/perception/traffic_light/base/pose.h"

namespace apollo {
namespace perception {
namespace traffic_light {

struct ProjectOption {
  explicit ProjectOption(const CameraId &id) : camera_id(id) {}

  CameraId camera_id;
};

/**
 * @brief Projection project traffic light(3D) to a region on the image(2D).
 *       The data(map & params) for projection should be load in init function.
 */
class BaseProjection {
 public:
  BaseProjection() = default;

  virtual ~BaseProjection() = default;

  /**
   * @brief project the traffic_light to a region on the image.
   * @params camera's coeffients
   * @params car's pose
   * @params option for project
   * @params in:traffic light 's x,y,z
   *         out:map info & the region on the image.
   */
  virtual bool Project(const CameraCoeffient &camera_coeffient,
                       const Eigen::Matrix4d &pose,
                       const apollo::hdmap::Signal &tl_info,
                       Light *light) const = 0;
};

REGISTER_REGISTERER(BaseProjection);
#define REGISTER_PROJECTION(name) REGISTER_CLASS(BaseProjection, name)

/**
 * @brief load transformation_matrix from file
 */
bool load_transformation_matrix_from_file(const std::string &file_name,
                                          Eigen::Matrix4d *matrix4d);

/**
 * @brief load Matrix4d from file
 */
bool load_matrix4d_from_file(const std::string &file_name,
                             const std::string &key, Eigen::Matrix4d *matrix);
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_TRAFFIC_LIGHT_INTERFACE_BASE_PROJECTION_H_
