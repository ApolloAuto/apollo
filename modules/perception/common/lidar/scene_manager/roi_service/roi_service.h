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
#include <vector>

#include "Eigen/Dense"

#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/common/lidar/scene_manager/scene_service.h"
#include "modules/perception/common/util.h"

namespace apollo {
namespace perception {
namespace lidar {

class ROIServiceContent : public SceneServiceContent {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  using Vec2ui = Eigen::Matrix<size_t, 2, 1>;
  enum class DirectionMajor { XMAJOR = 0, YMAJOR = 1 };

  ROIServiceContent() = default;
  ~ROIServiceContent() = default;

  // @brief: get a copy of this service content
  // @param [out]: copy of the service content
  void GetCopy(SceneServiceContent* content) const override;

  // @brief: set service content from outside
  // @param [in]: input service content
  void SetContent(const SceneServiceContent& content) override;

  // @brief: get service content name
  // @return: name
  std::string Name() const override { return "ROIServiceContent"; }

  // @brief: check bitmap bit
  // @brief return true if bit has been set
  inline bool CheckBit(const size_t loc, const uint64_t block) const;

  // @brief: check if point in bitmp;
  // @return: return true if point is in roi
  bool Check(const Eigen::Vector3d& world_point) const;

 public:
  std::vector<uint64_t> bitmap_;
  Vec2ui map_size_;
  double cell_size_ = 0.25;
  double range_ = 120.0;
  DirectionMajor major_dir_ = DirectionMajor::XMAJOR;
  Eigen::Vector3d transform_;
};

class ROIService : public SceneService {
 public:
  ROIService() = default;
  ~ROIService() { roi_content_ref_ = nullptr; }

  // @brief: initialize scene service
  // @param [in]: init options
  bool Init(const SceneServiceInitOptions& options =
                SceneServiceInitOptions()) override;
  // @brief: get service name
  // @return: name
  std::string Name() const override { return "ROIService"; }

 public:
  bool QueryIsPointInROI(const Eigen::Vector3d& world_point);
  bool QueryIsPointInROI(const Eigen::Vector3d& world_point,
                         const ROIServiceContent& content);

 protected:
  ROIServiceContent* roi_content_ref_ = nullptr;
};

typedef std::shared_ptr<ROIServiceContent> ROIServiceContentPtr;
typedef std::shared_ptr<const ROIServiceContent> ROIServiceContentConstPtr;

typedef std::shared_ptr<ROIService> ROIServicePtr;
typedef std::shared_ptr<const ROIService> ROIServiceConstPtr;

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
