/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/common/lidar/scene_manager/ground_service/ground_service.h"

#include <limits>

#include "cyber/common/file.h"
#include "modules/perception/common/algorithm/i_lib/geometry/i_plane.h"
#include "modules/perception/common/lidar/scene_manager/ground_service/proto/ground_service_config.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

using cyber::common::GetAbsolutePath;

void GroundServiceContent::GetCopy(SceneServiceContent* content) const {
  GroundServiceContent* ground_content =
      dynamic_cast<GroundServiceContent*>(content);
  if (ground_content == nullptr) {
    return;
  }
  ground_content->grid_ = grid_;
  ground_content->grid_center_ = grid_center_;
  ground_content->resolution_x_ = resolution_x_;
  ground_content->resolution_y_ = resolution_y_;
  ground_content->bound_x_min_ = bound_x_min_;
  ground_content->bound_y_min_ = bound_y_min_;
  ground_content->bound_x_max_ = bound_x_max_;
  ground_content->bound_y_max_ = bound_y_max_;
  ground_content->rows_ = rows_;
  ground_content->cols_ = cols_;
  ground_content->service_ready_ = service_ready_;
}

void GroundServiceContent::SetContent(const SceneServiceContent& content) {
  const GroundServiceContent* ground_content =
      dynamic_cast<const GroundServiceContent*>(&content);
  if (ground_content == nullptr) {
    return;
  }
  grid_ = ground_content->grid_;
  grid_center_ = ground_content->grid_center_;
  resolution_x_ = ground_content->resolution_x_;
  resolution_y_ = ground_content->resolution_y_;
  bound_x_min_ = ground_content->bound_x_min_;
  bound_y_min_ = ground_content->bound_y_min_;
  bound_x_max_ = ground_content->bound_x_max_;
  bound_y_max_ = ground_content->bound_y_max_;
  rows_ = ground_content->rows_;
  cols_ = ground_content->cols_;

  service_ready_ = true;
}

uint32_t inline GetIndex(uint32_t r, uint32_t c, uint32_t cols) {
  return (r * cols + c);
}

bool GroundServiceContent::PointToGrid(const Eigen::Vector3d& world_point,
                                       uint32_t* grid_index) const {
  double x = world_point(0) - grid_center_(0);
  double y = world_point(1) - grid_center_(1);
  if (x < bound_x_min_ || x > bound_x_max_ || y < bound_y_min_ ||
      y > bound_y_max_) {
    (*grid_index) = 0;
    return false;
  }
  uint32_t c = std::min(
      cols_ - 1, static_cast<uint32_t>((x - bound_x_min_) / resolution_x_));
  uint32_t r = std::min(
      rows_ - 1, static_cast<uint32_t>((y - bound_y_min_) / resolution_y_));
  (*grid_index) = GetIndex(r, c, cols_);
  return true;
}

float GroundServiceContent::PointToPlaneDistance(
    const Eigen::Vector3d& world_point) const {
  uint32_t grid_index = 0;
  if (!PointToGrid(world_point, &grid_index)) {
    return std::numeric_limits<float>::max();
  }

  float offset_pt[3];
  float params[4];
  float out = std::numeric_limits<float>::max();

  offset_pt[0] = static_cast<float>(world_point(0) - grid_center_(0));
  offset_pt[1] = static_cast<float>(world_point(1) - grid_center_(1));
  offset_pt[2] = static_cast<float>(world_point(2) - grid_center_(2));

  const GroundNode* node = grid_.DataPtr() + grid_index;
  if (node->confidence > 0.f) {
    params[0] = node->params(0);
    params[1] = node->params(1);
    params[2] = node->params(2);
    params[3] = node->params(3);
    out = algorithm::IPlaneToPointSignedDistanceWUnitNorm(params, offset_pt);
  }
  return out;
}

bool GroundServiceContent::Init(double roi_x, double roi_y, uint32_t rows,
                                uint32_t cols) {
  bound_x_min_ = -roi_x;
  bound_y_min_ = -roi_y;
  bound_x_max_ = roi_x;
  bound_y_max_ = roi_y;

  rows_ = rows;
  cols_ = cols;

  resolution_x_ = (bound_x_max_ - bound_x_min_) / cols_;
  resolution_y_ = (bound_y_max_ - bound_y_min_) / rows_;

  grid_.Init(rows_, cols_);
  return true;
}

bool GroundService::Init(const SceneServiceInitOptions& options) {
  self_content_.reset(new GroundServiceContent);
  ground_content_ref_ =
      dynamic_cast<GroundServiceContent*>(self_content_.get());
  // initialize ground service content from config manager and proto
  // including resolution, grid size, ...
  std::string config_file = FLAGS_ground_service_file;
  if (!options.config_file.empty()) {
    config_file = options.config_file;
  }

  config_file = GetCommonConfigFile(config_file);

  GroundServiceConfig config_params;
  ACHECK(cyber::common::GetProtoFromFile(config_file, &config_params))
      << "Failed to parse GroundServiceConfig config file.";

  double roi_region_rad_x = config_params.roi_rad_x();
  double roi_region_rad_y = config_params.roi_rad_y();
  uint32_t rows = config_params.grid_size();
  uint32_t cols = config_params.grid_size();

  ground_content_ref_->Init(roi_region_rad_x, roi_region_rad_y, rows, cols);

  return true;
}

float GroundService::QueryPointToGroundDistance(
    const Eigen::Vector3d& world_point) {
  std::lock_guard<std::mutex> lock(mutex_);
  float distance =
      QueryPointToGroundDistance(world_point, *ground_content_ref_);
  return distance;
}

float GroundService::QueryPointToGroundDistance(
    const Eigen::Vector3d& world_point, const GroundServiceContent& content) {
  return content.PointToPlaneDistance(world_point);
}

PERCEPTION_REGISTER_SCENESERVICECONTENT(GroundServiceContent);
PERCEPTION_REGISTER_SCENESERVICE(GroundService);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
