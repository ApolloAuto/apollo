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
#include "modules/perception/lidar/lib/scene_manager/roi_service/roi_service.h"

#include "cyber/common/file.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/lib/scene_manager/roi_service/proto/roi_service.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

using cyber::common::GetAbsolutePath;

void ROIServiceContent::GetCopy(SceneServiceContent* content) const {
  ROIServiceContent* roi_content = dynamic_cast<ROIServiceContent*>(content);
  if (roi_content == nullptr) {
    return;
  }
  roi_content->bitmap_ = bitmap_;
  roi_content->map_size_ = map_size_;
  roi_content->cell_size_ = cell_size_;
  roi_content->range_ = range_;
  roi_content->major_dir_ = major_dir_;
  roi_content->transform_ = transform_;
  roi_content->service_ready_ = service_ready_;
}

void ROIServiceContent::SetContent(const SceneServiceContent& content) {
  const ROIServiceContent* roi_content =
      dynamic_cast<const ROIServiceContent*>(&content);
  if (roi_content == nullptr) {
    return;
  }
  // copy content
  bitmap_ = roi_content->bitmap_;
  map_size_ = roi_content->map_size_;
  cell_size_ = roi_content->cell_size_;
  range_ = roi_content->range_;
  major_dir_ = roi_content->major_dir_;
  transform_ = roi_content->transform_;
  // set ready to true
  service_ready_ = true;
}

inline bool ROIServiceContent::CheckBit(const size_t loc,
                                        const uint64_t block) const {
  return block & (static_cast<uint64_t>(1) << loc);
}

bool ROIServiceContent::Check(const Eigen::Vector3d& world_point) const {
  if (!service_ready_) {
    return false;
  }
  Eigen::Vector2d pt;
  pt(0) = world_point(0) - transform_(0) + range_;
  pt(1) = world_point(1) - transform_(1) + range_;
  if (pt(0) >= 0 && pt(0) < 2 * range_ && pt(1) >= 0 && pt(1) < 2 * range_) {
    const size_t ix =
        static_cast<size_t>(pt(static_cast<size_t>(major_dir_)) / cell_size_);
    const size_t iy = static_cast<size_t>(
        pt(static_cast<size_t>(major_dir_) ^ 1) / cell_size_);
    const size_t idx_y = iy >> 6;
    const size_t idx_z = iy & 63;
    const size_t idx = ix * map_size_[1] + idx_y;
    return CheckBit(idx_z, bitmap_[idx]);
  }
  return false;
}

bool ROIService::Init(const SceneServiceInitOptions& options) {
  self_content_.reset(new ROIServiceContent);
  roi_content_ref_ = dynamic_cast<ROIServiceContent*>(self_content_.get());
  auto config_manager = lib::ConfigManager::Instance();
  const lib::ModelConfig* model_config = nullptr;
  ACHECK(config_manager->GetModelConfig(Name(), &model_config));
  const std::string work_root = config_manager->work_root();
  std::string config_file;
  std::string root_path;
  ACHECK(model_config->get_value("root_path", &root_path));
  config_file = GetAbsolutePath(work_root, root_path);
  config_file = GetAbsolutePath(config_file, "roi_service.conf");
  ROIServiceConfig config;
  ACHECK(cyber::common::GetProtoFromFile(config_file, &config));
  roi_content_ref_->cell_size_ = config.cell_size();
  roi_content_ref_->range_ = config.range();
  return true;
}

bool ROIService::QueryIsPointInROI(const Eigen::Vector3d& world_point) {
  std::lock_guard<std::mutex> lock(mutex_);
  bool status = QueryIsPointInROI(world_point, *roi_content_ref_);
  return status;
}

bool ROIService::QueryIsPointInROI(const Eigen::Vector3d& world_point,
                                   const ROIServiceContent& content) {
  return content.Check(world_point);
}

PERCEPTION_REGISTER_SCENESERVICECONTENT(ROIServiceContent);
PERCEPTION_REGISTER_SCENESERVICE(ROIService);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
