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
#pragma once

#include <memory>
#include <string>

#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/common/lidar/scene_manager/ground_service/ground_struct.h"
#include "modules/perception/common/lidar/scene_manager/scene_service.h"
#include "modules/perception/common/util.h"

namespace apollo {
namespace perception {
namespace lidar {

class GroundServiceContent : public SceneServiceContent {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  GroundServiceContent() = default;
  ~GroundServiceContent() = default;
  // @brief: get a copy of this service content
  // @param [out]: copy of the service content
  void GetCopy(SceneServiceContent* content) const override;
  // @brief: set service content from outside
  // @param [in]: input service content
  void SetContent(const SceneServiceContent& content) override;
  // @brief: get service content name
  // @return: name
  std::string Name() const override { return "GroundServiceContent"; }
  bool Init(double roi_x, double roi_y, uint32_t rows, uint32_t cols);

 public:
  // @brief: mapping function from point to grid index
  // @param [in]: point in world frame
  // @param [out] grid index
  // @return: false if exceeding the range
  bool PointToGrid(const Eigen::Vector3d& world_point,
                   uint32_t* grid_index) const;
  // @brief: compute distance of point to ground plane
  // @param [in]: point in world frame
  // @return: distance
  float PointToPlaneDistance(const Eigen::Vector3d& world_point) const;

 public:
  GroundGrid grid_;
  Eigen::Vector3d grid_center_;
  uint32_t rows_ = 0;  // y
  uint32_t cols_ = 0;  // x
  double resolution_x_ = 0.0;
  double resolution_y_ = 0.0;
  double bound_x_min_ = 0.0;
  double bound_y_min_ = 0.0;
  double bound_x_max_ = 0.0;
  double bound_y_max_ = 0.0;
};

class GroundService : public SceneService {
 public:
  GroundService() = default;
  ~GroundService() { ground_content_ref_ = nullptr; }
  // @brief: initialize scene service
  // @param [in]: init options
  bool Init(const SceneServiceInitOptions& options =
                SceneServiceInitOptions()) override;
  // @brief: get service name
  // @return: name
  std::string Name() const override { return "GroundService"; }

  GroundServiceContent* GetGroundServiceContent() const {
    return ground_content_ref_;
  }

 public:
  // @brief: Query point to ground distance
  // @param [in]: point in world frame
  // @return: distance
  float QueryPointToGroundDistance(const Eigen::Vector3d& world_point);
  // @brief: Query point to ground distance
  // @param [in]: point in world frame
  // @param [in]: outside service content
  // @return: distance
  float QueryPointToGroundDistance(const Eigen::Vector3d& world_point,
                                   const GroundServiceContent& content);

 protected:
  GroundServiceContent* ground_content_ref_ = nullptr;
};

typedef std::shared_ptr<GroundServiceContent> GroundServiceContentPtr;
typedef std::shared_ptr<const GroundServiceContent>
    GroundServiceContentConstPtr;

typedef std::shared_ptr<GroundService> GroundServicePtr;
typedef std::shared_ptr<const GroundService> GroundServiceConstPtr;

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
