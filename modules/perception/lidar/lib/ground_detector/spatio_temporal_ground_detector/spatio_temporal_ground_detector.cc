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

#include "modules/perception/lidar/lib/ground_detector/spatio_temporal_ground_detector/spatio_temporal_ground_detector.h"

#include "cyber/common/file.h"
#include "modules/perception/common/point_cloud_processing/common.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/common/lidar_log.h"
#include "modules/perception/lidar/common/lidar_point_label.h"
#include "modules/perception/lidar/lib/ground_detector/spatio_temporal_ground_detector/proto/spatio_temporal_ground_detector_config.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

using apollo::cyber::common::GetProtoFromFile;

bool SpatioTemporalGroundDetector::Init(
    const GroundDetectorInitOptions& options) {
  const lib::ModelConfig* model_config = nullptr;
  auto config_manager = lib::ConfigManager::Instance();
  ACHECK(config_manager->GetModelConfig("SpatioTemporalGroundDetector",
                                        &model_config))
      << "Failed to get model config: SpatioTemporalGroundDetector";

  const std::string& work_root = config_manager->work_root();
  std::string root_path;
  ACHECK(model_config->get_value("root_path", &root_path))
      << "Failed to get value of root_path.";
  std::string config_file;
  config_file = apollo::cyber::common::GetAbsolutePath(work_root, root_path);
  config_file = apollo::cyber::common::GetAbsolutePath(
      config_file, "spatio_temporal_ground_detector.conf");

  // get config params
  SpatioTemporalGroundDetectorConfig config_params;
  ACHECK(GetProtoFromFile(config_file, &config_params))
      << "Failed to parse SpatioTemporalGroundDetectorConfig config file.";
  ground_thres_ = config_params.ground_thres();
  use_roi_ = config_params.use_roi();
  use_ground_service_ = config_params.use_ground_service();

  param_ = new common::PlaneFitGroundDetectorParam;
  param_->roi_region_rad_x = config_params.roi_rad_x();
  param_->roi_region_rad_y = config_params.roi_rad_y();
  param_->roi_region_rad_z = config_params.roi_rad_z();
  param_->nr_grids_coarse = config_params.grid_size();
  param_->nr_smooth_iter = config_params.nr_smooth_iter();

  pfdetector_ = new common::PlaneFitGroundDetector(*param_);
  pfdetector_->Init();

  point_indices_temp_.resize(default_point_size_);
  data_.resize(default_point_size_ * 3);
  ground_height_signed_.resize(default_point_size_);

  ground_service_content_.Init(
      config_params.roi_rad_x(), config_params.roi_rad_y(),
      config_params.grid_size(), config_params.grid_size());
  return true;
}

bool SpatioTemporalGroundDetector::Detect(const GroundDetectorOptions& options,
                                          LidarFrame* frame) {
  // check input
  if (frame == nullptr) {
    AERROR << "Input null frame ptr.";
    return false;
  }
  if (frame->cloud.get() == nullptr || frame->world_cloud.get() == nullptr) {
    AERROR << "Input null frame cloud.";
    return false;
  }
  if (frame->cloud->empty() || frame->world_cloud->empty()) {
    AERROR << "Input none points.";
    return false;
  }

  unsigned int data_id = 0;
  unsigned int valid_point_num = 0;
  unsigned int valid_point_num_cur = 0;
  size_t i = 0;
  size_t num_points = 0;
  size_t num_points_all = 0;
  int index = 0;
  unsigned int nr_points_element = 3;
  float z_distance = 0.0f;

  cloud_center_(0) = frame->lidar2world_pose(0, 3);
  cloud_center_(1) = frame->lidar2world_pose(1, 3);
  cloud_center_(2) = frame->lidar2world_pose(2, 3);

  // check output
  frame->non_ground_indices.indices.clear();

  if (frame->roi_indices.indices.empty()) {
    use_roi_ = false;
  }

  if (use_roi_) {
    num_points = frame->roi_indices.indices.size();
  } else {
    num_points = frame->world_cloud->size();
  }

  ADEBUG << "spatial temporal seg: use roi " << use_roi_ << " num points "
         << num_points;

  // reallocate memory if points num > the preallocated size
  num_points_all = num_points;

  if (num_points_all > default_point_size_) {
    default_point_size_ = num_points * 2;
    point_indices_temp_.resize(default_point_size_);
    data_.resize(default_point_size_ * 3);
    ground_height_signed_.resize(default_point_size_);
  }

  // copy point data, filtering lower points under ground
  if (use_roi_) {
    for (i = 0; i < num_points; ++i) {
      index = frame->roi_indices.indices[i];
      const auto& pt = frame->world_cloud->at(index);
      point_indices_temp_[valid_point_num++] = index;
      data_[data_id++] = static_cast<float>(pt.x - cloud_center_(0));
      data_[data_id++] = static_cast<float>(pt.y - cloud_center_(1));
      data_[data_id++] = static_cast<float>(pt.z - cloud_center_(2));
    }
  } else {
    for (i = 0; i < num_points; ++i) {
      const auto& pt = frame->world_cloud->at(i);
      point_indices_temp_[valid_point_num++] = static_cast<int>(i);
      data_[data_id++] = static_cast<float>(pt.x - cloud_center_(0));
      data_[data_id++] = static_cast<float>(pt.y - cloud_center_(1));
      data_[data_id++] = static_cast<float>(pt.z - cloud_center_(2));
    }
  }

  valid_point_num_cur = valid_point_num;

  CHECK_EQ(data_id, valid_point_num * 3);
  base::PointIndices& non_ground_indices = frame->non_ground_indices;
  ADEBUG << "input of ground detector:" << valid_point_num;

  if (!pfdetector_->Detect(data_.data(), ground_height_signed_.data(),
                           valid_point_num, nr_points_element)) {
    ADEBUG << "failed to call ground detector!";
    non_ground_indices.indices.insert(
        non_ground_indices.indices.end(), point_indices_temp_.begin(),
        point_indices_temp_.begin() + valid_point_num);
    return false;
  }

  for (i = 0; i < valid_point_num_cur; ++i) {
    z_distance = ground_height_signed_.data()[i];
    frame->cloud->mutable_points_height()->at(point_indices_temp_[i]) =
        z_distance;
    frame->world_cloud->mutable_points_height()->at(point_indices_temp_[i]) =
        z_distance;
    if (common::IAbs(z_distance) > ground_thres_) {
      non_ground_indices.indices.push_back(point_indices_temp_[i]);
    } else {
      frame->cloud->mutable_points_label()->at(point_indices_temp_[i]) =
          static_cast<uint8_t>(LidarPointLabel::GROUND);
      frame->world_cloud->mutable_points_label()->at(point_indices_temp_[i]) =
          static_cast<uint8_t>(LidarPointLabel::GROUND);
    }
  }
  AINFO << "succeed to call ground detector with non ground points "
        << non_ground_indices.indices.size();

  if (use_ground_service_) {
    auto ground_service = SceneManager::Instance().Service("GroundService");
    if (ground_service != nullptr) {
      ground_service_content_.grid_center_ = cloud_center_;
      ground_service_content_.grid_.Reset();
      GroundNode* node_ptr = ground_service_content_.grid_.DataPtr();
      unsigned int rows = pfdetector_->GetGridDimY();
      unsigned int cols = pfdetector_->GetGridDimX();
      unsigned int index = 0;
      for (unsigned int r = 0; r < rows; ++r) {
        for (unsigned int c = 0; c < cols; ++c) {
          const common::GroundPlaneLiDAR* plane =
              pfdetector_->GetGroundPlane(r, c);
          if (plane->IsValid()) {
            index = r * cols + c;
            GroundNode* node = node_ptr + index;
            node->params(0) = plane->params[0];
            node->params(1) = plane->params[1];
            node->params(2) = plane->params[2];
            node->params(3) = plane->params[3];
            node->confidence = 1.0;
          }
        }
      }
      ground_service->UpdateServiceContent(ground_service_content_);
    } else {
      AINFO << "Failed to find ground service and cannot update.";
    }
  }
  return true;
}

PERCEPTION_REGISTER_GROUNDDETECTOR(SpatioTemporalGroundDetector);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
