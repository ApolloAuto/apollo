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
#include "modules/perception/traffic_light_region_proposal/preprocessor/multi_camera_projection.h"

#include <algorithm>
#include <limits>
#include <numeric>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/util/eigen_defs.h"
#include "modules/perception/common/camera/common/util.h"
#include "modules/perception/common/algorithm/io/io_util.h"
#include "modules/perception/common/algorithm/sensor_manager/sensor_manager.h"

namespace apollo {
namespace perception {
namespace trafficlight {

using apollo::common::EigenVector;

bool MultiCamerasProjection::Init(const MultiCamerasInitOption& options) {
  if (options.camera_names.empty()) {
    AERROR << "no cameras to be projected";
    return false;
  }
  algorithm::SensorManager* sensor_manager =
      algorithm::SensorManager::Instance();
  if (!sensor_manager->Init()) {
    AERROR << "sensor_manager init failed";
  }

  for (size_t i = 0; i < options.camera_names.size(); ++i) {
    const std::string& cur_camera_name = options.camera_names.at(i);
    AINFO << "init camera " << cur_camera_name;

    if (!sensor_manager->IsSensorExist(cur_camera_name)) {
      AERROR << "sensor " << cur_camera_name << " do not exist";
      return false;
    }
    camera_models_[cur_camera_name] =
        std::dynamic_pointer_cast<base::BrownCameraDistortionModel>(
            sensor_manager->GetDistortCameraModel(cur_camera_name));
    if (!(camera_models_[cur_camera_name])) {
      AERROR << "get null camera_model, camera_name: " << cur_camera_name;
      return false;
    }
    camera_names_.push_back(cur_camera_name);
  }
  // sort camera_names_ by focal lengths (descending order)
  std::sort(camera_names_.begin(), camera_names_.end(),
            [&](const std::string& lhs, const std::string& rhs) {
              const auto lhs_cam_intrinsics =
                  camera_models_[lhs]->get_intrinsic_params();
              const auto rhs_cam_intrinsics =
                  camera_models_[rhs]->get_intrinsic_params();
              auto lhs_focal_length =
                  0.5 * (lhs_cam_intrinsics(0, 0) + lhs_cam_intrinsics(1, 1));
              auto rhs_focal_length =
                  0.5 * (rhs_cam_intrinsics(0, 0) + rhs_cam_intrinsics(1, 1));
              return lhs_focal_length > rhs_focal_length;
            });
  AINFO << "camera_names sorted by descending focal lengths: "
        << std::accumulate(camera_names_.begin(), camera_names_.end(),
                           std::string(""),
                           [](std::string& sum, const std::string& s) {
                             return sum + s + " ";
                           });

  return true;
}

bool MultiCamerasProjection::Project(const camera::CarPose& pose,
                                     const ProjectOption& option,
                                     base::TrafficLight* light) const {
  if (!HasCamera(option.camera_name)) {
    AERROR << "no camera: " << option.camera_name;
    return false;
  }

  Eigen::Matrix4d c2w_pose;

  if (pose.c2w_poses_.find(option.camera_name) == pose.c2w_poses_.end()) {
    return false;
  }
  c2w_pose = pose.c2w_poses_.at(option.camera_name);

  bool ret = false;
  AINFO << "project use camera_name: " << option.camera_name;
  ret = BoundaryBasedProject(camera_models_.at(option.camera_name), c2w_pose,
                             light->region.points, light);

  if (!ret) {
    AWARN << "Projection failed projection the traffic light. "
          << "camera_name: " << option.camera_name;
    return false;
  }
  return true;
}

bool MultiCamerasProjection::HasCamera(const std::string& camera_name) const {
  auto iter =
      std::find(camera_names_.begin(), camera_names_.end(), camera_name);
  return iter != camera_names_.end() &&
         camera_models_.find(camera_name) != camera_models_.end();
}

int MultiCamerasProjection::getImageWidth(
    const std::string& camera_name) const {
  if (!HasCamera(camera_name)) {
    AERROR << "getImageWidth failed, camera_name: " << camera_name;
    return -1;
  }
  return static_cast<int>(camera_models_.at(camera_name)->get_width());
}

int MultiCamerasProjection::getImageHeight(
    const std::string& camera_name) const {
  if (!HasCamera(camera_name)) {
    AERROR << "getImageHeight failed, camera_name: " << camera_name;
    return -1;
  }
  return static_cast<int>(camera_models_.at(camera_name)->get_height());
}

bool MultiCamerasProjection::BoundaryBasedProject(
    const base::BrownCameraDistortionModelPtr camera_model,
    const Eigen::Matrix4d& c2w_pose,
    const std::vector<base::PointXYZID>& points,
    base::TrafficLight* light) const {
  if (camera_model.get() == nullptr) {
    AERROR << "camera_model is not available.";
    return false;
  }
  int width = static_cast<int>(camera_model->get_width());
  int height = static_cast<int>(camera_model->get_height());
  int bound_size = static_cast<int>(points.size());
  if (bound_size < 4) {
    AERROR << "invalid bound_size " << bound_size;
    return false;
  }
  EigenVector<Eigen::Vector2i> pts2d(bound_size);
  auto c2w_pose_inverse = c2w_pose.inverse();

  for (int i = 0; i < bound_size; ++i) {
    const auto& pt3d_world = points.at(i);
    Eigen::Vector3d pt3d_cam =
        (c2w_pose_inverse *
         Eigen::Vector4d(pt3d_world.x, pt3d_world.y, pt3d_world.z, 1.0))
            .head(3);
    if (std::islessequal(pt3d_cam[2], 0.0)) {
      AWARN << "light bound point behind the car: " << pt3d_cam;
      return false;
    }
    pts2d[i] = camera_model->Project(pt3d_cam.cast<float>()).cast<int>();
  }

  int min_x = std::numeric_limits<int>::max();
  int max_x = std::numeric_limits<int>::min();
  int min_y = std::numeric_limits<int>::max();
  int max_y = std::numeric_limits<int>::min();
  for (const auto& pt : pts2d) {
    min_x = std::min(pt[0], min_x);
    max_x = std::max(pt[0], max_x);
    min_y = std::min(pt[1], min_y);
    max_y = std::max(pt[1], max_y);
  }

  base::BBox2DI roi(min_x, min_y, max_x, max_y);
  if (camera::OutOfValidRegion(roi, width, height) || roi.Area() == 0) {
    AWARN << "Projection get ROI outside the image. ";
    return false;
  }
  light->region.projection_roi = base::RectI(roi);
  return true;
}

}  // namespace trafficlight
}  // namespace perception
}  // namespace apollo
